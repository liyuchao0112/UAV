#ifndef __UAV_BOOSTER_CONFIG_H__
#define __UAV_BOOSTER_CONFIG_H__

namespace uav_booster {

constexpr float FRIC_RADIUS{0.03f};
constexpr float FRIC_RADPS_TOLERANCE{100.0f}, FRIC_RADPS_DEADZONE{30.0f};
constexpr float FRIC_SHOOT_TORQUE_THRESHOLD{20.0f};

constexpr float TRIGGER_REDUCTION_RATIO{36.0f};
constexpr float TRIGGER_CONTINUOUS_RADPS{10.0f};
constexpr float TRIGGER_RAD_TOLERANCE{0.01f}, TRIGGER_RAD_DEADZONE{0.05f};

constexpr float CALI_REVERSE_RADPS{3.0f};
constexpr float CALI_FORWARD_RAD{0.522733748f+0.33131066f};

constexpr uint32_t BLOCK_TIME_THRESHOLD{1000};
constexpr float BLOCK_SPD_ERROR_RATE_THRESHOLD{0.5f};
constexpr float BLOCK_RAD_THRESHOLD{pyro::PI / 16.0f}, BLOCK_SPD_THRESHOLD{0.3f};

constexpr float TARGET_BULLET_SPEED{18.5f};

constexpr float SINGLE_BULLET_RAD{pyro::PI / 4.0f};

} // namespace uav_booster

#endif