#include "pyro_uav_booster.h"

namespace pyro {

void uav_booster_t::fsm_active_t::state_ready_t::enter(owner *owner) {
    //防跳变
    owner->_ctx.data.target_trigger_rad = owner->_ctx.data.current_trigger_rad;
    owner->_ctx.data.target_trigger_radps = 0;
}

void uav_booster_t::fsm_active_t::state_ready_t::execute(owner *owner) {
    if(!owner->_ctx.cmd->is_fric_on || !_is_fric_ready(&owner->_ctx))
        request_switch(&owner->_active_state._waiting_state);
    else {
        if(owner->_ctx.cmd->fire_licence && owner->_ctx.cmd->single_shoot)
            request_switch(&owner->_active_state._single_state);
    
        if(owner->_ctx.cmd->fire_licence && owner->_ctx.cmd->continue_shoot)
            request_switch(&owner->_active_state._continue_state);
    }
    
    _trigger_control(&owner->_ctx);
    _send_trigger_command(&owner->_ctx);
}

void uav_booster_t::fsm_active_t::state_ready_t::exit(owner *owner) {}

} // namespace pyro