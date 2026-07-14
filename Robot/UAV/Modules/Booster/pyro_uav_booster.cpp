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

    _ctx.data.current_trigger_rad =
        loop_fp32_constrain(_ctx.cfg.motor.trigger->get_current_position() - uav_booster::TRIGGER_OFFSET, -PI, PI);
    _ctx.data.current_trigger_radps = _ctx.cfg.motor.trigger->get_current_rotate();
}

void uav_booster_t::_fsm_execute() {
    _ctx.cmd = &_current_cmd;
    
    if(_ctx.cmd->mode == cmd_base_t::mode_t::PASSIVE)
        _main_fsm.change_state(&_passive_state);
    else if(_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
        _main_fsm.change_state(&_active_state);

    _main_fsm.execute(this);
}

} // namespace pyro