#ifndef __PYRO_UAV_BOOSTER_H__
#define __PYRO_UAV_BOOSTER_H__

#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"

#include "uav_booster_config.h"

namespace pyro {

struct uav_booster_cmd_t final : public cmd_base_t {
    bool is_enable; //发射机构是否启用，主要调试用，代码中不使用

    bool is_fric_on;            // 摩擦轮是否开启
    bool single_shoot;          // 触发单发
    bool continue_shoot;        // 触发连发
    bool heat_control_on{false}; // 热量控制开关，false时跳过所有热量限制（调试用）
    bool fire_licence{};        // 发射许可，为false时拨弹盘绝对不允许转动

    uav_booster_cmd_t() : is_enable(false), is_fric_on(false), single_shoot(false), continue_shoot(false),
        fire_licence(false) {}
};

struct uav_booster_cfg_t {
    struct motor_cfg_t {
        motor_base_t *fric[2]{nullptr};
        motor_base_t *trigger{nullptr};
    };

    struct pid_cfg_t {
        pid_t *bullet_spd_pid{nullptr};
        pid_t *fric_pid[2]{nullptr};
        pid_t *trigger_pos_pid{nullptr};
        pid_t *trigger_spd_pid{nullptr};
    };

    motor_cfg_t motor;
    pid_cfg_t pid;
};

class uav_booster_t final 
        : public module_base_t<uav_booster_t, uav_booster_cmd_t, uav_booster_cfg_t> {
    friend class module_base_t;
    
    struct data_ctx_t;
    struct booster_ctx_t;

  public:
    uav_booster_t(const uav_booster_t&) = delete;
    uav_booster_t &operator=(const uav_booster_t&) = delete;
    
  private:
    uav_booster_t();
    ~uav_booster_t() override =default;

    // --- 基类接口 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 私有方法 ---
    static bool _is_fric_ready(booster_ctx_t *ctx);
    static void _fric_control(booster_ctx_t *ctx);
    static void _trigger_control(booster_ctx_t *ctx);
    static void _send_fric_command(booster_ctx_t *ctx);
    static void _send_trigger_command(booster_ctx_t *ctx);

    // --- 成员变量 ---

    // --- 运行时数据 ---
    struct data_ctx_t {
        float target_fric_radps[2]{0.0f};
        float target_trigger_rad{0.0f}, target_trigger_radps{0.0f};

        float current_fric_radps[2]{0.0f};
        float current_trigger_rad, current_trigger_radps{0.0f};
        float current_trigger_torque;

        float last_trigger_rad;
        
        float out_fric_torque[2]{0.0f};
        float out_trigger_torque{0.0f};

        enum class trigger_pid_mode_e {
            POS,
            SPD
        } trigger_mode{trigger_pid_mode_e::POS};
    };

    struct booster_ctx_t {
        uav_booster_cfg_t cfg;
        data_ctx_t data;
        uav_booster_cmd_t *cmd{};
    };

    booster_ctx_t _ctx;

    // --- FSM 状态定义 ---
    using owner = uav_booster_t;

    struct state_passive_t : public state_t<owner> {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct fsm_active_t : public fsm_t<owner> {
        struct state_waiting_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_ready_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_single_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_continue_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;           
        };

        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;
        
      private:
        state_waiting_t _waiting_state;
        state_ready_t _ready_state;
        state_single_t _single_state;
        state_continue_t _continue_state;
    };

    state_passive_t _passive_state;
    fsm_active_t _active_state;
    fsm_t<owner> _main_fsm;
};


} // namespace pyro


#endif