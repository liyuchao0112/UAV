#include "pyro_uav_booster.h"

namespace pyro {

void uav_booster_t::fsm_active_t::state_cali_forward_t::enter(owner *owner) {
    owner->_ctx.data.trigger_mode = uav_booster_t::data_ctx_t::trigger_pid_mode_e::POS;
    owner->_ctx.data.target_trigger_rad += uav_booster::CALI_FORWARD_RAD;
    owner->_ctx.data.block_start_tick = 0;
}

void uav_booster_t::fsm_active_t::state_cali_forward_t::execute(owner *owner) {
    if(!owner->_ctx.cmd->is_fric_on) {
        request_switch(&owner->_active_state._waiting_state);

        owner->_ctx.data.target_trigger_rad = owner->_ctx.data.current_trigger_rad;
        _trigger_control(&owner->_ctx);
        _send_trigger_command(&owner->_ctx);

        return;
    }

    _trigger_control(&owner->_ctx);
    _send_trigger_command(&owner->_ctx);

    //位置环时的堵转检测
    if(std::fabs(owner->_ctx.data.current_trigger_rad - owner->_ctx.data.target_trigger_rad) 
            > uav_booster::BLOCK_RAD_THRESHOLD &&
        std::abs(owner->_ctx.data.current_trigger_radps) 
            < uav_booster::BLOCK_SPD_THRESHOLD) {
        if(owner->_ctx.data.block_start_tick == 0)
            owner->_ctx.data.block_start_tick = xTaskGetTickCount();
        else if(xTaskGetTickCount() - owner->_ctx.data.block_start_tick
                >= pdMS_TO_TICKS(uav_booster::BLOCK_TIME_THRESHOLD)) {
            owner->_ctx.data.is_calibrated = false;
            request_switch(&owner->_active_state._cali_reverse_state);
        }
    } else {
        owner->_ctx.data.block_start_tick = 0;
    }

    if(std::fabs(owner->_ctx.data.current_trigger_rad - owner->_ctx.data.target_trigger_rad)
            < uav_booster::TRIGGER_RAD_TOLERANCE) {
        owner->_ctx.data.is_calibrated = true;

        if(_is_fric_ready(&owner->_ctx))
            request_switch(&owner->_active_state._ready_state);
        else
            request_switch(&owner->_active_state._waiting_state);
    }
}

void uav_booster_t::fsm_active_t::state_cali_forward_t::exit(owner *owner) {}

} // namespace pyro