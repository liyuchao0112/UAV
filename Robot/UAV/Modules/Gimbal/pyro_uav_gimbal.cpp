#include "pyro_uav_gimbal.h"
#include "pyro_ins.h"
#include "pyro_algo_common.h"
#include <algorithm>

namespace pyro {

uav_gimbal_t::uav_gimbal_t()
        : module_base_t("uav_gimbal", 512, 512, task_base_t::priority_t::HIGH) {
    _ctx.data = {};
}

status_t uav_gimbal_t::_init() {
    _ctx.cfg = _module_deps;
    return PYRO_OK;
}

void uav_gimbal_t::_update_feedback() {
    //imu反馈数据
    ins_drv_t::get_instance()->get_rads_n(&_ctx.data.current_imu_yaw_rad,
        &_ctx.data.current_imu_pitch_rad, &_ctx.data.current_imu_roll_rad);
    ins_drv_t::get_instance()->get_gyro_n(&_ctx.data.current_imu_yaw_radps,
        &_ctx.data.current_imu_pitch_radps, &_ctx.data.current_imu_roll_radps);

    //电机反馈数据
    _ctx.cfg.motor_cfg.pitch->update_feedback();
    _ctx.cfg.motor_cfg.yaw->update_feedback();
    
    //电机数据处理
    _ctx.data.current_motor_pitch_rad =
        wrap2pi_f32(_ctx.cfg.motor_cfg.pitch->get_current_position() - uav_gimbal::PITCH_MOTOR_OFFSET);
    _ctx.data.current_motor_pitch_radps =
        _ctx.cfg.motor_cfg.pitch->get_current_rotate();
    
    _ctx.data.current_motor_yaw_rad =
        loop_fp32_constrain(_ctx.cfg.motor_cfg.yaw->get_current_position() - uav_gimbal::YAW_MOTOR_OFFSET, -PI, PI);
    _ctx.data.current_motor_yaw_radps =
        _ctx.cfg.motor_cfg.yaw->get_current_rotate();
}

void uav_gimbal_t::_fsm_execute() {
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_passive_state);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_active_state);

    _main_fsm.execute(this);
}

void uav_gimbal_t::_mec_control(gimbal_ctx_t *ctx) {
    //pitch位置环
    ctx->data.target_pitch_radps =
        ctx->cfg.pid_cfg.pitch_pos_pid->calculate(
            ctx->data.target_pitch_rad, ctx->data.current_motor_pitch_rad);

    //打了几个点 ai拟合的
    // 2. 双项正弦高精度拟合 (傅里叶级数)
    float term1 = 0.455f * sinf(2.12f * ctx->data.current_motor_pitch_rad - 0.58f);
    float term2 = 0.038f * sinf(10.5f * ctx->data.current_motor_pitch_rad + 1.85f);

    ctx->data.gravity_compensate = -0.762f + term1 + term2;
    
    //pitch速度环
    ctx->data.out_pitch_torque =
        ctx->cfg.pid_cfg.pitch_spd_pid->calculate(
            ctx->data.target_pitch_radps, ctx->data.current_motor_pitch_radps)
        + ctx->data.gravity_compensate; //重力补偿（使用了imu的数据）
    
    ctx->data.out_pitch_torque = std::clamp(ctx->data.out_pitch_torque,
        uav_gimbal::PITCH_MIN_MOTOR_TORQUE, uav_gimbal::PITCH_MAX_MOTOR_TORQUE);

    //yaw位置环
    ctx->data.target_yaw_radps =
        ctx->cfg.pid_cfg.yaw_pos_pid->calculate(
            ctx->data.target_yaw_rad, ctx->data.current_motor_yaw_rad);
    
    //yaw速度环
    ctx->data.out_yaw_torque =
        ctx->cfg.pid_cfg.yaw_spd_pid->calculate(
            ctx->data.target_yaw_radps, ctx->data.current_motor_yaw_radps);

    ctx->data.out_yaw_torque = std::clamp(ctx->data.out_yaw_torque,
        uav_gimbal::YAW_MIN_MOTOR_TORQUE, uav_gimbal::YAW_MAX_MOTOR_TORQUE);
}

void uav_gimbal_t::_imu_control(gimbal_ctx_t *ctx) {
    //pitch位置环
    ctx->data.target_pitch_radps =
        ctx->cfg.pid_cfg.pitch_pos_pid->calculate(
            ctx->data.target_pitch_rad, ctx->data.current_imu_pitch_rad);
    
    //pitch速度环
    ctx->data.out_pitch_torque =
        ctx->cfg.pid_cfg.pitch_spd_pid->calculate(
            ctx->data.target_pitch_radps, ctx->data.current_imu_pitch_radps)
        - uav_gimbal::GRAVITY_OFFSET * cos(ctx->data.current_imu_pitch_rad); //重力补偿（使用了imu的数据）

    //yaw位置环
    ctx->data.target_yaw_radps =
        ctx->cfg.pid_cfg.yaw_pos_pid->calculate(
            ctx->data.target_yaw_rad, ctx->data.current_imu_yaw_rad);
    
    //yaw速度环
    ctx->data.out_yaw_torque =
        ctx->cfg.pid_cfg.yaw_spd_pid->calculate(
            ctx->data.target_yaw_radps, ctx->data.current_imu_yaw_radps);    
}

void uav_gimbal_t::_send_motor_command(gimbal_ctx_t *ctx) {
    ctx->cfg.motor_cfg.pitch->send_torque(ctx->data.out_pitch_torque);
    ctx->cfg.motor_cfg.yaw->send_torque(ctx->data.out_yaw_torque);
}

} // namespace pyro