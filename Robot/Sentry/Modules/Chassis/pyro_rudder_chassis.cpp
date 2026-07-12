#include "pyro_rudder_chassis.h"
#include "pyro_algo_common.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_referee.h"

namespace pyro {

rud_chassis_t::rud_chassis_t()
        : module_base_t("rudder", 512, 512, task_base_t::priority_t::HIGH) {
    _ctx.data  = {};
    debug_data = {};
}

status_t rud_chassis_t::_init() {
    _kinematics               = new rudder_kin_t(0.36f, 0.36f);
    _ctx.rud_config           = _module_deps;
    _ctx.hardware.power_meter = new powermeter_drv_t(0x212, can_hub_t::can2); //未使用
    _ctx.power.data           = new powermeter_data();
    return PYRO_OK;
}

void rud_chassis_t::_update_feedback() {
    _ctx.rud_config.motor.rudder[0]->update_feedback();
    _ctx.rud_config.motor.rudder[1]->update_feedback();
    _ctx.rud_config.motor.rudder[2]->update_feedback();
    _ctx.rud_config.motor.rudder[3]->update_feedback();
    _ctx.rud_config.motor.wheel[0]->update_feedback();
    _ctx.rud_config.motor.wheel[1]->update_feedback();
    _ctx.rud_config.motor.wheel[2]->update_feedback();
    _ctx.rud_config.motor.wheel[3]->update_feedback();

    //舵机位置
    _ctx.data.current_states.modules[rudder_kin_t::FL].angle =
        _ctx.rud_config.motor.rudder[0]->get_current_position() -
        rudder::RUD_POS_OFFSET[0];
    _ctx.data.current_states.modules[rudder_kin_t::FR].angle =
        _ctx.rud_config.motor.rudder[1]->get_current_position() -
        rudder::RUD_POS_OFFSET[1];
    _ctx.data.current_states.modules[rudder_kin_t::BL].angle =
        _ctx.rud_config.motor.rudder[2]->get_current_position() -
        rudder::RUD_POS_OFFSET[2];
    _ctx.data.current_states.modules[rudder_kin_t::BR].angle =
        _ctx.rud_config.motor.rudder[3]->get_current_position() -
        rudder::RUD_POS_OFFSET[3];
    
    //归入+/-2*pi范围
    for(int i=0;i<4;i++)
        _ctx.data.current_states.modules[i].angle=wrap2pi_f32(_ctx.data.current_states.modules[i].angle);

    //舵机角速度
    _ctx.data.current_rud_radps[0] =
        _ctx.rud_config.motor.rudder[0]->get_current_rotate();
    _ctx.data.current_rud_radps[1] =
        _ctx.rud_config.motor.rudder[1]->get_current_rotate();
    _ctx.data.current_rud_radps[2] =
        _ctx.rud_config.motor.rudder[2]->get_current_rotate();
    _ctx.data.current_rud_radps[3] =
        _ctx.rud_config.motor.rudder[3]->get_current_rotate();
    
    //轮速
    _ctx.data.current_states.modules[rudder_kin_t::FL].speed =
        _ctx.rud_config.motor.wheel[0]->get_current_rotate() *
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio * rudder::WHEEL_RADIUS;
    _ctx.data.current_states.modules[rudder_kin_t::FR].speed =
        _ctx.rud_config.motor.wheel[1]->get_current_rotate() *
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio * rudder::WHEEL_RADIUS;
    _ctx.data.current_states.modules[rudder_kin_t::BL].speed =
        _ctx.rud_config.motor.wheel[2]->get_current_rotate() *
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio * rudder::WHEEL_RADIUS;
    _ctx.data.current_states.modules[rudder_kin_t::BR].speed =
        _ctx.rud_config.motor.wheel[3]->get_current_rotate() *
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio * rudder::WHEEL_RADIUS;

    //更新 cap_tx 数据
    _ctx.supercap_cmd.power_referee     = 0; //？ 应该未使用
    _ctx.supercap_cmd.power_limit_referee =
        referee_drv_t::get_instance()->get_data().robot_status.chassis_power_limit;
    _ctx.supercap_cmd.power_buffer_limit_referee = rudder::CAP_BUFFER_LIMIT_REFEREE;
    _ctx.supercap_cmd.power_buffer_referee =
        referee_drv_t::get_instance()->get_data().power_heat.buffer_energy;
    _ctx.supercap_cmd.use_cap           = 1;
    _ctx.supercap_cmd.kill_chassis_user = 0;
    _ctx.supercap_cmd.speed_up_user_now = 0;
    
    //更新 cap_rx 数据
    _ctx.cap_feedback = supercap_drv_t::get_instance()->get_feedback();
}

void rud_chassis_t::_kinematics_solve() {
    if (_ctx.cmd->mode == rud_cmd_t::mode_t::PASSIVE) {
        _ctx.cmd->vx        = 0.0f;
        _ctx.cmd->vy        = 0.0f;
        _ctx.cmd->wz        = 0.0f;
        _ctx.cmd->yaw_error = 0.0f;
    } 
    if(_ctx.cmd->mode == rud_cmd_t::mode_t::ACTIVE) {
        if(_ctx.cmd->follow_yaw == true)
            _ctx.cmd->wz = _ctx.rud_config.pid.follow_yaw_pid->calculate(0, _ctx.cmd->yaw_error);
        if(_ctx.cmd->follow_yaw == false)
        {
            const float vx = _ctx.cmd->vx;
            const float vy = _ctx.cmd->vy;
            _ctx.cmd->vx =
                vx * cosf(_ctx.cmd->yaw_error) - vy * sinf(_ctx.cmd->yaw_error);
            _ctx.cmd->vy =
                vx * sinf(_ctx.cmd->yaw_error) + vy * cosf(_ctx.cmd->yaw_error);
        }
    }

    _ctx.data.target_states = _kinematics->solve(
        _ctx.cmd->vx, _ctx.cmd->vy, _ctx.cmd->wz, _ctx.data.current_states);
}

void rud_chassis_t::_chassis_control(rud_ctx_t *ctx) {
    for(int i=0;i<4;i++) {
        const float rud_pos_output =
            ctx->rud_config.pid.rud_pos_pid[i]->calculate(
                ctx->data.target_states.modules[i].angle,
                ctx->data.current_states.modules[i].angle);

        // 舵速度环
        ctx->data.out_rud_torque[i] =
            ctx->rud_config.pid.rud_spd_pid[i]->calculate(
                rud_pos_output, ctx->data.current_rud_radps[i]);

        // 轮子速度环
        ctx->data.out_wheel_torque[i] =
            ctx->rud_config.pid.wheel_pid[i]->calculate(
                ctx->data.target_states.modules[i].speed,
                ctx->data.current_states.modules[i].speed);
    }

#if POWER_CONTROL_USE

    std::array<power_control_drv_t::motor_data_t, rudder::POWERCONTROL_NUM> motor_data;
    
    power_control_drv_t &power_controller = power_control_drv_t::get_instance();
    for (int i = 0; i < rudder::POWERCONTROL_NUM; i++) {
        motor_data.at(i).gyro = ctx->data.current_states.modules[i].speed;
        motor_data.at(i).torque_cmd = ctx->data.out_wheel_torque[i];
        motor_data.at(i).power_predict = power_controller.motor_power_predict(
            i, motor_data.at(i).torque_cmd, motor_data.at(i).gyro);
    }
    float power_limit = referee_drv_t::get_instance()->get_data().robot_status.chassis_power_limit;

    if (ctx->cap_feedback.vot_cap >= 1000) {
        // 平均分配
        power_controller.calculate_restricted_torques(
            motor_data.data(), 4, rudder::POWERCONTROL_NUM, power_limit + 100);
    } else {
        // 平均分配
        power_controller.calculate_restricted_torques(
            motor_data.data(), 4, rudder::POWERCONTROL_NUM, power_limit);
    }

    for (int i = 0; i < rudder::POWERCONTROL_NUM; i++)
        ctx->data.out_wheel_torque[i] = motor_data.at(i).restricted_torque;

#endif
}

void rud_chassis_t::_send_motor_command(rud_ctx_t *ctx) {
    // 发送舵机扭矩命令
    for (int i = 0; i < 4; i++)
        ctx->rud_config.motor.rudder[i]->send_torque(
            ctx->data.out_rud_torque[i]);

    // 发送轮子扭矩命令
    for (int i = 0; i < 4; i++)
        // test_ctorque[i] = ctx->data.out_wheel_torque[i];
        ctx->rud_config.motor.wheel[i]->send_torque(
            ctx->data.out_wheel_torque[i]);
}

void rud_chassis_t::_send_supercap_command() const {
    supercap_drv_t::get_instance()->send_cmd(_ctx.supercap_cmd); // NOLINT
}

}
