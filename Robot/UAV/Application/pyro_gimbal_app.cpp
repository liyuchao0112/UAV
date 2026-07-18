#include "pyro_core_config.h"
#include "pyro_core_def.h"

#include "pyro_module_base.h"
#include "pyro_uav_gimbal.h"
#include "pyro_mutex.h"
#include "pyro_rc_base_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_com_cantx.h"
#include "pyro_com_canrx.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"

using namespace pyro;

uav_gimbal_cmd_t *gimbal_cmd_ptr = nullptr;
uav_gimbal_cfg_t *gimbal_cfg_ptr = nullptr;
uav_gimbal_t *gimbal_ptr = nullptr;

static TaskHandle_t gimbal_task_handle = nullptr;

// virtual_rc_t d_vrc;

void gimbal_config() {
    gimbal_cfg_ptr->motor_cfg.pitch = new dm_motor_drv_t(0x01,0x00,can_hub_t::can2);
    gimbal_cfg_ptr->motor_cfg.yaw = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_5,can_hub_t::can2);

    static_cast<dm_motor_drv_t *>(gimbal_cfg_ptr->motor_cfg.pitch)->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(gimbal_cfg_ptr->motor_cfg.pitch)
        ->set_rotate_range(uav_gimbal::PITCH_MIN_MOTOR_RADPS, uav_gimbal::PITCH_MAX_MOTOR_RADPS);
    static_cast<dm_motor_drv_t *>(gimbal_cfg_ptr->motor_cfg.pitch)
        ->set_torque_range(uav_gimbal::PITCH_MIN_MOTOR_TORQUE, uav_gimbal::PITCH_MAX_MOTOR_TORQUE);

    //没写跟踪微分器，先空着
    
    //pid
    gimbal_cfg_ptr->pid_cfg.yaw_pos_pid = new pid_t(15.5f, 0.0f, 0.0f, 0.0f, 10.0f, 50, 20, 4);
    gimbal_cfg_ptr->pid_cfg.yaw_spd_pid = new pid_t(0.3f, 0.08f, 0.0003f, 1.5f, 3.0f, 50, 20, 4);
    gimbal_cfg_ptr->pid_cfg.pitch_pos_pid = new pid_t(20.2f, 0.0004f, 0.006f, 0.4f, 9.0f, 50, 30, 4);
    gimbal_cfg_ptr->pid_cfg.pitch_spd_pid = new pid_t(1.18f, 0.068f, 0.006f, 1.8f, 7.0f, 30, 15, 4);
}

void gimbal_vt032cmd(uint32_t notify_val) {
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if(!gimbal_cmd_ptr->is_enable) {
        gimbal_cmd_ptr->mode = uav_gimbal_cmd_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->target_pitch_delta_angle = 0.0f;
        gimbal_cmd_ptr->target_yaw_delta_angle = 0.0f;

        return;
    }

    if(vrc.switches.gear.current_pos == pyro::sw_pos_t::UP) {
        gimbal_cmd_ptr->mode = uav_gimbal_cmd_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->target_pitch_delta_angle = 0.0f;
        gimbal_cmd_ptr->target_yaw_delta_angle = 0.0f;
    }
    else if(vrc.switches.gear.current_pos == pyro::sw_pos_t::MID) {
        gimbal_cmd_ptr->mode = uav_gimbal_cmd_t::mode_t::ACTIVE;
        gimbal_cmd_ptr->target_pitch_delta_angle = - vrc.axes.ry * uav_gimbal::RC_PITCH_COEFFICIENT;
        gimbal_cmd_ptr->target_yaw_delta_angle = vrc.axes.rx * uav_gimbal::RC_YAW_COEFFICIENT;
    }
    else if(vrc.switches.gear.current_pos == pyro::sw_pos_t::DOWN) {
        //AUTO，没写，现在是无力状态
        gimbal_cmd_ptr->mode = uav_gimbal_cmd_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->target_pitch_delta_angle = 0.0f;
        gimbal_cmd_ptr->target_yaw_delta_angle = 0.0f;
    }
}

void gimbal_dr162cmd(uint32_t notify_val) {
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    // d_vrc = vrc;

    if(!gimbal_cmd_ptr->is_enable) {
        gimbal_cmd_ptr->mode = uav_gimbal_cmd_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->target_pitch_delta_angle = 0.0f;
        gimbal_cmd_ptr->target_yaw_delta_angle = 0.0f;

        return;
    }

    if(vrc.switches.right.current_pos == pyro::sw_pos_t::UP) {
        gimbal_cmd_ptr->mode = uav_gimbal_cmd_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->target_pitch_delta_angle = 0.0f;
        gimbal_cmd_ptr->target_yaw_delta_angle = 0.0f;
    }
    else if(vrc.switches.right.current_pos == pyro::sw_pos_t::MID) {
        gimbal_cmd_ptr->mode = uav_gimbal_cmd_t::mode_t::ACTIVE;
        gimbal_cmd_ptr->target_pitch_delta_angle = - vrc.axes.ry * uav_gimbal::RC_PITCH_COEFFICIENT;
        gimbal_cmd_ptr->target_yaw_delta_angle = vrc.axes.rx * uav_gimbal::RC_YAW_COEFFICIENT;
    }
    else if(vrc.switches.right.current_pos == pyro::sw_pos_t::DOWN) {
        gimbal_cmd_ptr->mode = uav_gimbal_cmd_t::mode_t::ACTIVE;
        gimbal_cmd_ptr->target_pitch_delta_angle = - vrc.axes.ry * uav_gimbal::RC_PITCH_COEFFICIENT;
        gimbal_cmd_ptr->target_yaw_delta_angle = vrc.axes.rx * uav_gimbal::RC_YAW_COEFFICIENT;
    }
}

extern "C" {
    void uav_gimbal_thread(void *argument) {
        while(true) {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

            if (vt03_drv_t::instance().check_online()) {
                gimbal_vt032cmd(notify_val);
            }
            else if(dr16_drv_t::instance().check_online()) {
                gimbal_dr162cmd(notify_val);
            }
            else
                gimbal_cmd_ptr->mode=uav_gimbal_cmd_t::mode_t::PASSIVE;

            gimbal_ptr->set_command(*gimbal_cmd_ptr);
            
            vTaskDelay(1);
        }
    }

    void uav_gimbal_init(void *argument) {
        gimbal_cmd_ptr = new uav_gimbal_cmd_t();
        gimbal_cfg_ptr = new uav_gimbal_cfg_t();
        gimbal_ptr = uav_gimbal_t::instance();

        gimbal_cmd_ptr->is_enable = true;

        gimbal_config();
        gimbal_ptr->configure(*gimbal_cfg_ptr);
        gimbal_ptr->start();

        xTaskCreate(uav_gimbal_thread, "uav_gimbal_thread", 256, nullptr,
                    configMAX_PRIORITIES - 1, &gimbal_task_handle);
        
        vTaskDelete(nullptr);
    }
}