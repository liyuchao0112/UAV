#include "pyro_uav_gimbal.h"

namespace pyro {

void uav_gimbal_t::state_passive_t::enter(owner *owner) {
    owner->_ctx.cfg.pid_cfg.pitch_pos_pid->clear();
    owner->_ctx.cfg.pid_cfg.pitch_spd_pid->clear();
    owner->_ctx.cfg.pid_cfg.yaw_pos_pid->clear();
    owner->_ctx.cfg.pid_cfg.yaw_spd_pid->clear();

    owner->_ctx.cfg.motor_cfg.pitch->disable();
    owner->_ctx.cfg.motor_cfg.yaw->disable();
}

void uav_gimbal_t::state_passive_t::execute(owner *owner) {
    owner->_ctx.data.out_pitch_torque = 0;
    owner->_ctx.data.out_yaw_torque = 0;

    //防跳变
    if(owner->_ctx.cmd->is_imu_control) {
        owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_imu_pitch_rad;
        owner->_ctx.data.target_yaw_rad = owner->_ctx.data.current_imu_yaw_rad;
    } else {
        owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_motor_pitch_rad;
        owner->_ctx.data.target_yaw_rad = owner->_ctx.data.current_motor_yaw_rad;
    }

    uav_gimbal_t::_send_motor_command(&owner->_ctx);
}

void uav_gimbal_t::state_passive_t::exit(owner *owner) {}

} // namespace pyro