#include "pyro_uav_booster.h"

extern pyro::uav_booster_cmd_t *booster_cmd_ptr;

namespace pyro {

void uav_booster_t::fsm_active_t::state_single_t::enter(owner *owner) {
    booster_cmd_ptr->single_shoot = false;

    owner->_ctx.data.trigger_mode = uav_booster_t::data_ctx_t::trigger_pid_mode_e::SPD;
    owner->_ctx.data.target_trigger_radps = 10.0f * uav_booster::TRIGGER_REDUCTION_RATIO;  // 固定转速

    last_trigger_rad = owner->_ctx.data.current_trigger_rad;
    total_trigger_rotate_rad = 0.0f;
}

void uav_booster_t::fsm_active_t::state_single_t::execute(owner *owner) {
    if(!owner->_ctx.cmd->is_fric_on) {
        request_switch(&owner->_active_state._waiting_state);

        owner->_ctx.data.target_trigger_rad = owner->_ctx.data.current_trigger_rad;
        _trigger_control(&owner->_ctx);
        _send_trigger_command(&owner->_ctx);
    } else {
        // 累积真实旋转量（带过零 wrapping）
        float delta = owner->_ctx.data.current_trigger_rad - last_trigger_rad;
        if(delta > PI)
            delta -= 2.0f * PI;
        else if(delta < -PI)
            delta += 2.0f * PI;
        total_trigger_rotate_rad += delta;
        last_trigger_rad = owner->_ctx.data.current_trigger_rad;

        _trigger_control(&owner->_ctx);
        _send_trigger_command(&owner->_ctx);

        // 达到 9*PI 后退出
        constexpr float target_rad = PI / 4.0f * uav_booster::TRIGGER_REDUCTION_RATIO;
        if (std::fabs(total_trigger_rotate_rad) >= target_rad
                - uav_booster::TRIGGER_RAD_TOLERANCE * uav_booster::TRIGGER_REDUCTION_RATIO) {
            if (_is_fric_ready(&owner->_ctx))
                request_switch(&owner->_active_state._ready_state);
            else
                request_switch(&owner->_active_state._waiting_state);
        }

        // float delta = owner->_ctx.data.current_trigger_rad - last_trigger_rad;
        // if (delta > PI)
        //     delta -= 2*PI;
        // else if (delta < -PI) 
        //     delta += 2*PI;
        
        // total_trigger_rotate_rad += delta;
        
        // last_trigger_rad = owner->_ctx.data.current_trigger_rad;

        // owner->_ctx.data.target_trigger_rad += PI / 4.0f;

        // _trigger_control(&owner->_ctx);
        // _send_trigger_command(&owner->_ctx);

        // if(std::fabs(total_trigger_rotate_rad - PI / 4.0f * uav_booster::TRIGGER_REDUCTION_RATIO)
        //         < uav_booster::TRIGGER_RAD_TOLERANCE * uav_booster::TRIGGER_REDUCTION_RATIO) {
        //     if(_is_fric_ready(&owner->_ctx))
        //         request_switch(&owner->_active_state._ready_state);
        //     else
        //         request_switch(&owner->_active_state._waiting_state);
        // }
    }
}

void uav_booster_t::fsm_active_t::state_single_t::exit(owner *owner) {}

} // namespace pyro