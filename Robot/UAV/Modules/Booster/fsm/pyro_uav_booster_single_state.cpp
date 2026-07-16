#include "pyro_uav_booster.h"

namespace pyro {

void uav_booster_t::fsm_active_t::state_single_t::enter(owner *owner) {
    owner->_ctx.cmd->single_shoot = false;
    
    owner->_ctx.data.trigger_mode = uav_booster_t::data_ctx_t::trigger_pid_mode_e::POS;

    owner->_ctx.data.target_trigger_rad += PI / 4.0f;

    //处理过零点问题
    const float error = owner->_ctx.data.target_trigger_rad - owner->_ctx.data.current_trigger_rad;
    if (error > PI)
        owner->_ctx.data.target_trigger_rad -= 2.0f * PI;
    else if (error < -PI)
        owner->_ctx.data.target_trigger_rad += 2.0f * PI;
}

void uav_booster_t::fsm_active_t::state_single_t::execute(owner *owner) {
    if(!owner->_ctx.cmd->is_fric_on) {
        request_switch(&owner->_active_state._waiting_state);

        owner->_ctx.data.target_trigger_rad = owner->_ctx.data.current_trigger_rad;
        _trigger_control(&owner->_ctx);
        _send_trigger_command(&owner->_ctx);
    } else {
        _trigger_control(&owner->_ctx);
        _send_trigger_command(&owner->_ctx);

        if(std::fabs(owner->_ctx.data.target_trigger_rad - owner->_ctx.data.current_trigger_rad)
                < uav_booster::TRIGGER_RAD_TOLERANCE) {
            if(_is_fric_ready(&owner->_ctx))
                request_switch(&owner->_active_state._ready_state);
            else
                request_switch(&owner->_active_state._waiting_state);
        }
    }
}

void uav_booster_t::fsm_active_t::state_single_t::exit(owner *owner) {}

} // namespace pyro