#include "pyro_uav_gimbal.h"
#include "pyro_algo_common.h"
#include <algorithm>

namespace pyro {

void uav_gimbal_t::state_active_t::enter(owner *owner) {
    //防跳变
    if(owner->_ctx.cmd->is_imu_control) {
        owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_imu_pitch_rad;
        owner->_ctx.data.target_yaw_rad = owner->_ctx.data.current_imu_yaw_rad;
    } else {
        owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_motor_pitch_rad;
        owner->_ctx.data.target_yaw_rad = owner->_ctx.data.current_motor_yaw_rad;
    }

    owner->_ctx.cfg.pid_cfg.pitch_pos_pid->clear();
    owner->_ctx.cfg.pid_cfg.pitch_spd_pid->clear();
    owner->_ctx.cfg.pid_cfg.yaw_pos_pid->clear();
    owner->_ctx.cfg.pid_cfg.yaw_spd_pid->clear();

    owner->_ctx.cfg.motor_cfg.pitch->enable();
    owner->_ctx.cfg.motor_cfg.yaw->enable();
}

void uav_gimbal_t::state_active_t::execute(owner *owner) {
    owner->_ctx.data.target_pitch_rad += owner->_ctx.cmd->target_pitch_delta_angle;
    owner->_ctx.data.target_yaw_rad += owner->_ctx.cmd->target_yaw_delta_angle;

    if(owner->_ctx.cmd->is_imu_control) {
        //imu懒得写了，直接置零吧
        owner->_ctx.data.out_yaw_torque = 0;
        owner->_ctx.data.out_pitch_torque = 0;
    } else {
        //按电机机械角度控制来限幅
        owner->_ctx.data.target_pitch_rad = std::clamp(
            owner->_ctx.data.target_pitch_rad, uav_gimbal::PITCH_MIN_MOTOR_RAD, uav_gimbal::PITCH_MAX_MOTOR_RAD);
        owner->_ctx.data.target_yaw_rad = std::clamp(
            owner->_ctx.data.target_yaw_rad, uav_gimbal::YAW_MIN_MOTOR_RAD, uav_gimbal::YAW_MAX_MOTOR_RAD);

        //角度超限，理论上不会
        const float yaw_error = owner->_ctx.data.target_yaw_rad - owner->_ctx.data.current_motor_yaw_rad;
        if (yaw_error > PI)
            owner->_ctx.data.target_yaw_rad -= 2.0f * PI;
        else if (yaw_error < -PI)
            owner->_ctx.data.target_yaw_rad += 2.0f * PI;
        
        _mec_control(&owner->_ctx);
    }

    _send_motor_command(&owner->_ctx);
}

void uav_gimbal_t::state_active_t::exit(owner *owner) {}

} // namespace pyro