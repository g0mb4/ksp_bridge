#include "ksp_bridge/ksp_bridge.hpp"
#include "ksp_bridge/utils.hpp"

void KSPBridge::throttle_sub(const ksp_bridge_interfaces::msg::Float::SharedPtr msg)
{
    if (msg->value > 1 || msg->value < 0) {
        RCLCPP_WARN(get_logger(), "Throttle must be between 0 and 1, current value is %f, will be clamped.", msg->value);
    }

    float value = clamp<float>(msg->value, 0, 1);

    try {
        m_vessel->control().set_throttle(value);
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
    }
}