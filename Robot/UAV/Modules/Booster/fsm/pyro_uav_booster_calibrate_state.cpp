#include "pyro_uav_booster.h"

namespace pyro {

void uav_booster_t::fsm_active_t::state_calibrate_t::enter(owner *owner) {
    owner->_ctx.data.trigger_mode = uav_booster_t::data_ctx_t::trigger_pid_mode_e::SPD;
    owner->_ctx.data.target_trigger_radps = - uav_booster::TRIGGER_CONTINUOUS_RADPS;

    
}

void uav_booster_t::fsm_active_t::state_calibrate_t::execute(owner *owner) {

}

void uav_booster_t::fsm_active_t::state_calibrate_t::exit(owner *owner) {

}

} // namespace pyro