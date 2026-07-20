#include "pyro_uav_booster.h"
#include "pyro_algo_common.h"

namespace pyro {

uav_booster_t::uav_booster_t()
        : module_base_t("uav_booster", 512, 512, task_base_t::priority_t::HIGH) {
    _ctx.data={};
}

status_t uav_booster_t::_init() {
    _ctx.cfg = _module_deps;
    return PYRO_OK;
}

void uav_booster_t::_update_feedback() {
    _ctx.cfg.motor.fric[0]->update_feedback();
    _ctx.cfg.motor.fric[1]->update_feedback();
    _ctx.cfg.motor.trigger->update_feedback();

    _ctx.data.current_fric_radps[0] = _ctx.cfg.motor.fric[0]->get_current_rotate();
    _ctx.data.current_fric_radps[1] = _ctx.cfg.motor.fric[1]->get_current_rotate();

    _ctx.data.current_trigger_radps =
        _ctx.cfg.motor.trigger->get_current_rotate() / uav_booster::TRIGGER_REDUCTION_RATIO;

    _ctx.data.current_trigger_torque = _ctx.cfg.motor.trigger->get_current_torque();

    const float now_trigger_rad = _ctx.cfg.motor.trigger->get_current_position();

    float delta_rad = now_trigger_rad - _ctx.data.last_trigger_rad;
    if (delta_rad > PI)
        delta_rad -= 2.0f * PI;
    else if (delta_rad < -PI)
        delta_rad += 2.0f * PI;

    _ctx.data.current_trigger_rad +=delta_rad / uav_booster::TRIGGER_REDUCTION_RATIO;

    _ctx.data.last_trigger_rad = now_trigger_rad;
}

void uav_booster_t::_fsm_execute() {
    _ctx.cmd = &_current_cmd;
    
    if(_ctx.cmd->mode == cmd_base_t::mode_t::PASSIVE)
        _main_fsm.change_state(&_passive_state);
    else if(_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
        _main_fsm.change_state(&_active_state);

    _main_fsm.execute(this);
}

bool uav_booster_t::_is_fric_ready(booster_ctx_t *ctx) {
    return (std::fabs(ctx->data.current_fric_radps[0])
            - uav_booster::TARGET_BULLET_SPEED / uav_booster::FRIC_RADIUS) < uav_booster::FRIC_RADPS_TOLERANCE
        && (std::fabs(ctx->data.current_fric_radps[1])
            - uav_booster::TARGET_BULLET_SPEED / uav_booster::FRIC_RADIUS) < uav_booster::FRIC_RADPS_TOLERANCE;
}

void uav_booster_t::_fric_control(booster_ctx_t *ctx) {
    ctx->data.out_fric_torque[0] =
        ctx->cfg.pid.fric_pid[0]->calculate(ctx->data.target_fric_radps[0], ctx->data.current_fric_radps[0]);
    ctx->data.out_fric_torque[1] =
        ctx->cfg.pid.fric_pid[1]->calculate(ctx->data.target_fric_radps[1], ctx->data.current_fric_radps[1]);
}

void uav_booster_t::_trigger_control(booster_ctx_t *ctx) {
    if(ctx->data.trigger_mode == uav_booster_t::data_ctx_t::trigger_pid_mode_e::POS) {
        //处理过零点问题
        const float error = ctx->data.target_trigger_rad - ctx->data.current_trigger_rad;
        if (error > PI)
            ctx->data.target_trigger_rad -= 2.0f * PI;
        else if (error < -PI)
            ctx->data.target_trigger_rad += 2.0f * PI;

        // // 死区，防止由于安装间隙导致的振动
        // if(std::fabs(ctx->data.target_trigger_rad - ctx->data.current_trigger_rad)
        //         < uav_booster::TRIGGER_RAD_DEADZONE * uav_booster::TRIGGER_REDUCTION_RATIO ) {
        //     ctx->data.target_trigger_radps = 0.0f;
        //     ctx->data.out_trigger_torque =0.0f;
        //     return;
        // }

        ctx->data.target_trigger_radps =
            ctx->cfg.pid.trigger_pos_pid->calculate(ctx->data.target_trigger_rad, ctx->data.current_trigger_rad);
        ctx->data.out_trigger_torque =
            ctx->cfg.pid.trigger_spd_pid->calculate(ctx->data.target_trigger_radps, ctx->data.current_trigger_radps);
    }
    if(ctx->data.trigger_mode == uav_booster_t::data_ctx_t::trigger_pid_mode_e::SPD) {
        ctx->data.out_trigger_torque =
            ctx->cfg.pid.trigger_spd_pid->calculate(ctx->data.target_trigger_radps, ctx->data.current_trigger_radps);
    }
}

void uav_booster_t::_send_fric_command(booster_ctx_t *ctx) {
    ctx->cfg.motor.fric[0]->send_torque(ctx->data.out_fric_torque[0]);
    ctx->cfg.motor.fric[1]->send_torque(ctx->data.out_fric_torque[1]);
}

void uav_booster_t::_send_trigger_command(booster_ctx_t *ctx) {
    ctx->cfg.motor.trigger->send_torque(ctx->data.out_trigger_torque);
}

} // namespace pyro