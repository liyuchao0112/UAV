#ifndef __UAV_BOOSTER_CONFIG_H__
#define __UAV_BOOSTER_CONFIG_H__

namespace uav_booster {

constexpr float FRIC_RADIUS{0.03f};
constexpr float FRIC_RADPS_TOLERANCE{100.0f}, FRIC_RADPS_DEADZONE{10.0f};

constexpr float TRIGGER_CONTINUOUS_RADPS{10.0f};
constexpr float TRIGGER_RAD_TOLERANCE{0.01f};

constexpr float TARGET_BULLET_SPEED{18.5f};

} // namespace uav_booster

#endif