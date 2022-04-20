#include <ksp_bridge/ksp_bridge.hpp>
#include <ksp_bridge/utils.hpp>

void KSPBridge::cmd_throttle_sub(const ksp_bridge_interfaces::msg::CmdThrottle::SharedPtr msg)
{
    if (msg->throttle > 1 || msg->throttle < 0) {
        RCLCPP_WARN(get_logger(), "Throttle must be between 0 and 1, current value is %f, will be clamped.", msg->throttle);
    }

    float value = clamp<float>(msg->throttle, 0, 1);

    try {
        m_vessel->control().set_throttle(value);
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", __FILE__, __LINE__, ex.what());
    }
}

void KSPBridge::cmd_rotation_sub(const ksp_bridge_interfaces::msg::CmdRotation::SharedPtr msg)
{
    if (msg->pitch > 1 || msg->pitch < -1) {
        RCLCPP_WARN(get_logger(), "Pitch must be between -1 and 1, current value is %f, will be clamped.", msg->pitch);
    }

    float value = clamp<float>(msg->pitch, -1, 1);

    try {
        m_vessel->control().set_pitch(value);
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", __FILE__, __LINE__, ex.what());
    }

    if (msg->yaw > 1 || msg->yaw < -1) {
        RCLCPP_WARN(get_logger(), "Yaw must be between -1 and 1, current value is %f, will be clamped.", msg->yaw);
    }

    value = clamp<float>(msg->yaw, -1, 1);

    try {
        m_vessel->control().set_yaw(value);
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", __FILE__, __LINE__, ex.what());
    }

    if (msg->roll > 1 || msg->roll < -1) {
        RCLCPP_WARN(get_logger(), "Roll must be between -1 and 1, current value is %f, will be clamped.", msg->roll);
    }

    value = clamp<float>(msg->roll, -1, 1);

    try {
        m_vessel->control().set_roll(value);
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", __FILE__, __LINE__, ex.what());
    }
}