//
// Created by Clean on 2026/4/6.
//
#include "pyro_screw_gimbal.h"
#include "screw_config.h"
#include "pyro_algo_common.h"
#include <algorithm>

namespace pyro
{
void screw_gimbal_t::fsm_active_t::autoaim_state_t::enter(owner *owner)
{
    // 切换到 Autoaim 模式时，使用 IMU 数据对齐目标防止跳变
    owner->_ctx.data.target_yaw_rad = owner->_ctx.data.yaw_imu_rad;
    owner->_ctx.data.target_pitch_rad = owner->_ctx.data.pitch_imu_rad;

    // 清除自瞄专用的 Pitch PID 以及 Yaw PID 的历史积分
    owner->_ctx.pid.pitch_autoaim_pos->clear();
    owner->_ctx.pid.pitch_autoaim_spd->clear();
    owner->_ctx.pid.yaw_pos->clear();
    owner->_ctx.pid.yaw_spd->clear();
}

void screw_gimbal_t::fsm_active_t::autoaim_state_t::execute(owner *owner)
{
    // 自瞄模式下，直接读取外部传入的绝对目标角度
    owner->_ctx.data.target_pitch_rad = owner->_ctx.cmd->target_pitch;
    owner->_ctx.data.target_yaw_rad = owner->_ctx.cmd->target_yaw;

    // ==========================================
    // 1. Pitch IMU 绝对限幅
    // ==========================================
    owner->_ctx.data.target_pitch_rad = std::clamp(
        owner->_ctx.data.target_pitch_rad, PITCH_MIN_IMU_RAD, PITCH_MAX_IMU_RAD);

    // ==========================================
    // 2. Yaw 环化限幅 (-PI, PI)
    // ==========================================
    owner->_ctx.data.target_yaw_rad =
        pyro::loop_fp32_constrain(owner->_ctx.data.target_yaw_rad, -PI, PI);

    // ==========================================
    // 3. 执行自瞄底层控制与发送指令
    // ==========================================
    owner->_gimbal_autoaim_control();
    screw_gimbal_t::_send_motor_command(&owner->_ctx);
}

void screw_gimbal_t::fsm_active_t::autoaim_state_t::exit(owner *owner)
{
}
} // namespace pyro