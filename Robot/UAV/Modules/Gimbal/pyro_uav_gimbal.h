#ifndef __PYRO_UAV_GIMBAL_H__
#define __PYRO_UAV_GIMBAL_H__

#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"

#include "uav_gimbal_config.h"

namespace pyro {

struct uav_gimbal_cmd_t final : public cmd_base_t {
    bool is_enable; //云台是否启用，主要调试用，代码中不使用

    //由控制器产生的期望值
    float target_pitch_angle, target_yaw_angle;
    float target_pitch_delta_angle, target_yaw_delta_angle; //位置环模拟移动速度

    bool is_imu_control;

    uav_gimbal_cmd_t() 
        : is_enable(false), target_pitch_angle(0.0f), target_yaw_angle(0.0f),
            target_pitch_delta_angle(0.0f), target_yaw_delta_angle(0.0f), is_imu_control(false) {}
};

struct uav_gimbal_cfg_t {
    struct motor_cfg_t {
        motor_base_t *pitch{nullptr};
        motor_base_t *yaw{nullptr};
    };

    struct pid_cfg_t {
        pid_t *pitch_pos_pid{nullptr};
        pid_t *pitch_spd_pid{nullptr};
        pid_t *yaw_pos_pid{nullptr};
        pid_t *yaw_spd_pid{nullptr};
    };

    motor_cfg_t motor_cfg;
    pid_cfg_t pid_cfg;
};

class uav_gimbal_t final
        : public module_base_t<uav_gimbal_t, uav_gimbal_cmd_t, uav_gimbal_cfg_t> {
    friend class module_base_t;

    struct data_ctx_t;
    struct gimbal_ctx_t;

  public:
    uav_gimbal_t(const uav_gimbal_t&) = delete;
    uav_gimbal_t &operator=(const uav_gimbal_t&) = delete;

  private:
    uav_gimbal_t();
    ~uav_gimbal_t() override = default;

    // --- 基类接口实现 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 私有辅助方法 ---
    static void _mec_control(gimbal_ctx_t *ctx);
    static void _imu_control(gimbal_ctx_t *ctx);
    static void _send_motor_command(gimbal_ctx_t *ctx);
    

    // --- 成员变量 ---
    
    //运行时数据
    struct data_ctx_t {
        //当前状态（电机反馈）
        float current_motor_pitch_rad{0.0f};
        float current_motor_pitch_radps{0.0f};
        float current_motor_yaw_rad{0.0f};
        float current_motor_yaw_radps{0.0f};
        float current_motor_roll_rad{0.0f};
        float current_motor_roll_radps{0.0f};

        //当前状态（imu反馈）
        float current_imu_pitch_rad{0.0f};
        float current_imu_pitch_radps{0.0f};
        float current_imu_yaw_rad{0.0f};
        float current_imu_yaw_radps{0.0f};
        float current_imu_roll_rad{0.0f};
        float current_imu_roll_radps{0.0f};

        //计算后的目标值
        float target_pitch_rad{0.0f};
        float target_pitch_radps{0.0f};
        float target_yaw_rad{0.0f};
        float target_yaw_radps{0.0f};

        //输出
        float out_pitch_torque{0.0f};
        float out_yaw_torque{0.0f};
    };

    struct gimbal_ctx_t {
        uav_gimbal_cfg_t cfg;
        data_ctx_t data;
        uav_gimbal_cmd_t *cmd{};
    };

    gimbal_ctx_t _ctx;

    // --- FSM 状态定义 ---
    using owner = uav_gimbal_t;

    struct state_passive_t : public state_t<owner> {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct state_active_t : public state_t<owner> {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    state_passive_t _passive_state;
    state_active_t _active_state;
    fsm_t<owner> _main_fsm;
};

} // namespace pyro

#endif