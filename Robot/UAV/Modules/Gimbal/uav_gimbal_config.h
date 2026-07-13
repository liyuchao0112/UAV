#ifndef __UAV_GIMBAL_CONFIG_H__
#define __UAV_GIMBAL_CONFIG_H__

namespace uav_gimbal {

constexpr float PITCH_MOTOR_OFFSET{0.0f};
constexpr float GRAVITY_OFFSET{0.95f};
constexpr float YAW_MOTOR_OFFSET{2.60700035f};
constexpr float PITCH_MAX_MOTOR_RAD{0.635f}, PITCH_MIN_MOTOR_RAD{-0.23f};
constexpr float PITCH_MAX_MOTOR_RADPS{30.0f}, PITCH_MIN_MOTOR_RADPS{-30.0f};
constexpr float PITCH_MAX_MOTOR_TORQUE{7.0f}, PITCH_MIN_MOTOR_TORQUE{-7.0f};
constexpr float YAW_MAX_MOTOR_RAD{1.5f}, YAW_MIN_MOTOR_RAD{-0.9f};

constexpr float RC_PITCH_COEFFICIENT{0.005f}, RC_YAW_COEFFICIENT{0.002f};

}

#endif