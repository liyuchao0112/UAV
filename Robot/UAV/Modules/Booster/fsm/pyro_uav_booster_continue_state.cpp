#include "pyro_uav_booster.h"

namespace pyro {

void uav_booster_t::fsm_active_t::state_continue_t::enter(owner *owner) {
    owner->_ctx.data.trigger_mode = uav_booster_t::data_ctx_t::trigger_pid_mode_e::SPD;

    owner->_ctx.data.target_trigger_radps = uav_booster::TRIGGER_CONTINUOUS_RADPS;
}

void uav_booster_t::fsm_active_t::state_continue_t::execute(owner *owner) {
    if(!owner->_ctx.cmd->is_fric_on) {
        request_switch(&owner->_active_state._waiting_state);

        owner->_ctx.data.target_trigger_rad = owner->_ctx.data.current_trigger_rad;
        _trigger_control(&owner->_ctx);
        _send_trigger_command(&owner->_ctx);
    } else {
        _trigger_control(&owner->_ctx);
        _send_trigger_command(&owner->_ctx);

        if(!owner->_ctx.cmd->fire_licence || !owner->_ctx.cmd->continue_shoot) {
            if(_is_fric_ready(&owner->_ctx))
                request_switch(&owner->_active_state._ready_state);
            else
                request_switch(&owner->_active_state._waiting_state);
        }
    }
}

void uav_booster_t::fsm_active_t::state_continue_t::exit(owner *owner) {
    owner->_ctx.data.trigger_mode = uav_booster_t::data_ctx_t::trigger_pid_mode_e::POS;
}

} // namespace pyro