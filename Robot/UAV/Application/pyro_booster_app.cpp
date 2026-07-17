#include "pyro_uav_booster.h"
#include "pyro_rc_base_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_module_base.h"
#include "pyro_dji_motor_drv.h"

using namespace pyro;

// 定义任务通知的位掩码 (Event Bits)
// VT03
constexpr uint32_t EVENT_BIT_FRIC_TOGGLE = (1 << 0);
constexpr uint32_t EVENT_BIT_SINGLE_FIRE = (1 << 1);
constexpr uint32_t EVENT_BIT_CONTINUE_FIRE = (1 << 2);
constexpr uint32_t EVENT_BIT_CONTINUE_END = (1 << 3);

//DR16
constexpr uint32_t EVENT_BIT_FRIC_ON = (1 << 4);
constexpr uint32_t EVENT_BIT_FRIC_OFF = (1 << 5);
constexpr uint32_t EVENT_BIT_FIRE = (1 << 6);
constexpr uint32_t EVENT_BIT_FIRE_END = (1 << 7);

uav_booster_cmd_t *booster_cmd_ptr = nullptr;
uav_booster_cfg_t *booster_cfg_ptr = nullptr;
uav_booster_t *booster_ptr = nullptr;

static TaskHandle_t booster_task_handle = nullptr;

virtual_rc_t d_vrc;

void booster_config() {
    //摩擦轮电机初始化
    booster_cfg_ptr->motor.fric[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,can_hub_t::can1);
    booster_cfg_ptr->motor.fric[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,can_hub_t::can1);
    //拨弹盘电机初始化
    booster_cfg_ptr->motor.trigger = new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_4,can_hub_t::can1);

    //摩擦轮pid初始化
    booster_cfg_ptr->pid.fric_pid[0] = new pid_t(0.3f, 0.0f, 0.0f, 1.0f, 20, 60, 15, 4);
    booster_cfg_ptr->pid.fric_pid[1] = new pid_t(0.3f, 0.0f, 0.0f, 1.0f, 20, 60, 15, 4);

    //拨弹盘pid初始化
    booster_cfg_ptr->pid.trigger_pos_pid = new pid_t(5.8f, 0.0f, 0.0f, 1.0f, 15.0f, 60, 30, 4);
    booster_cfg_ptr->pid.trigger_spd_pid = new pid_t(0.15f, 0.001f, 0.0f, 1.0f, 15.0f, 60, 30, 4);
}

void booster_vt032cmd(uint32_t notify_val) {
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if(!booster_cmd_ptr->is_enable) {
        booster_cmd_ptr->mode = uav_booster_cmd_t::mode_t::PASSIVE;
        booster_cmd_ptr->is_fric_on = false;
        booster_cmd_ptr->fire_licence = false;

        return;
    }

    if(vrc.switches.gear.current_pos == pyro::sw_pos_t::UP ||
            vrc.switches.gear.current_pos == pyro::sw_pos_t::DOWN) {
        booster_cmd_ptr->mode = uav_booster_cmd_t::mode_t::PASSIVE;
        booster_cmd_ptr->is_fric_on = false;
        booster_cmd_ptr->fire_licence = false;
    }
    if(vrc.switches.gear.current_pos == pyro::sw_pos_t::MID) {
        booster_cmd_ptr->mode = uav_booster_cmd_t::mode_t::ACTIVE;
        booster_cmd_ptr->fire_licence = true; //没接热量管理，所以临时直接授予发射许可

        if(notify_val & EVENT_BIT_FRIC_TOGGLE)
            booster_cmd_ptr->is_fric_on = !booster_cmd_ptr->is_fric_on;
        
        if(notify_val & EVENT_BIT_SINGLE_FIRE) {
            booster_cmd_ptr->is_fric_on = true;
            booster_cmd_ptr->single_shoot = true;
        }
        else if(notify_val & EVENT_BIT_CONTINUE_FIRE) {
            booster_cmd_ptr->is_fric_on = true;
            booster_cmd_ptr->continue_shoot =true;
        }

        if(notify_val & EVENT_BIT_CONTINUE_END)
            booster_cmd_ptr->continue_shoot = false;
    }
}

void booster_dr162cmd(uint32_t notify_val) {
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if(!booster_cmd_ptr->is_enable) {
        booster_cmd_ptr->mode = uav_booster_cmd_t::mode_t::PASSIVE;
        booster_cmd_ptr->is_fric_on = false;
        booster_cmd_ptr->fire_licence = false;

        return;
    }

    if(vrc.switches.right.current_pos == pyro::sw_pos_t::UP) {
        booster_cmd_ptr->mode = uav_booster_cmd_t::mode_t::PASSIVE;
        booster_cmd_ptr->is_fric_on = false;
        booster_cmd_ptr->fire_licence = false;
    }
    if(vrc.switches.right.current_pos == pyro::sw_pos_t::MID) {
        booster_cmd_ptr->mode = uav_booster_cmd_t::mode_t::ACTIVE;
        booster_cmd_ptr->fire_licence = true; //没接热量管理，所以临时直接授予发射许可

        if(notify_val & EVENT_BIT_FRIC_ON)
            booster_cmd_ptr->is_fric_on = true;
        
        if(notify_val & EVENT_BIT_FRIC_OFF)
            booster_cmd_ptr->is_fric_on = false;

        if(notify_val & EVENT_BIT_FIRE) {
            booster_cmd_ptr->is_fric_on = true;
            booster_cmd_ptr->single_shoot = true;
        }
        
        if(notify_val & EVENT_BIT_FIRE_END) {
            booster_cmd_ptr->single_shoot = false;
            booster_cmd_ptr->continue_shoot = false;
        }
    }
    if(vrc.switches.right.current_pos == pyro::sw_pos_t::DOWN) {
        booster_cmd_ptr->mode = uav_booster_cmd_t::mode_t::ACTIVE;
        booster_cmd_ptr->fire_licence = true; //没接热量管理，所以临时直接授予发射许可

        if(notify_val & EVENT_BIT_FRIC_ON)
            booster_cmd_ptr->is_fric_on = true;
        
        if(notify_val & EVENT_BIT_FRIC_OFF)
            booster_cmd_ptr->is_fric_on = false;

        if(notify_val & EVENT_BIT_FIRE) {
            booster_cmd_ptr->is_fric_on = true;
            booster_cmd_ptr->continue_shoot = true;
        }
        
        if(notify_val & EVENT_BIT_FIRE_END) {
            booster_cmd_ptr->single_shoot = false;
            booster_cmd_ptr->continue_shoot = false;
        }
    }
}

extern "C" {
    void uav_booster_thread(void *argument) {
        while(true) {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

            if(vt03_drv_t::instance().check_online()) {
                booster_vt032cmd(notify_val);
            }
            else if(dr16_drv_t::instance().check_online()) {
                booster_dr162cmd(notify_val);
            }
            else
                booster_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;

            booster_ptr->set_command(*booster_cmd_ptr);

            vTaskDelay(1);
        }
    }

    void uav_booster_init(void *argument) {
        booster_cmd_ptr = new uav_booster_cmd_t();
        booster_cfg_ptr = new uav_booster_cfg_t();
        booster_ptr = uav_booster_t::instance();

        booster_cmd_ptr->is_enable = true;

        booster_config();
        booster_ptr->configure(*booster_cfg_ptr);
        booster_ptr->start();

        xTaskCreate(uav_booster_thread, "uav_booster_thread", 256, nullptr,
                    configMAX_PRIORITIES - 1, &booster_task_handle);
        
        auto &vrc = pyro::rc_drv_t::read();
        
        d_vrc = vrc;

        // --- VT03 按键绑定 ---
        pyro::btn_broker::subscribe(&vrc.buttons.fn_l, pyro::btn_event_t::PRESS_DOWN,
            booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::SINGLE_CLICK,
            booster_task_handle, EVENT_BIT_SINGLE_FIRE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::LONG_PRESS_START,
            booster_task_handle, EVENT_BIT_CONTINUE_FIRE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::PRESS_UP,
            booster_task_handle, EVENT_BIT_CONTINUE_END);

        // --- DR16 按键绑定 ---
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::UP_TO_MID,
            booster_task_handle, EVENT_BIT_FRIC_ON);
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::MID_TO_UP,
            booster_task_handle, EVENT_BIT_FRIC_OFF);
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::MID_TO_DOWN,
            booster_task_handle, EVENT_BIT_FIRE);
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::DOWN_TO_MID,
            booster_task_handle, EVENT_BIT_FIRE_END);
        
        vTaskDelete(nullptr);
    }
}