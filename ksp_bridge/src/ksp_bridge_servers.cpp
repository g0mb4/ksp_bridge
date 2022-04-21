#include <ksp_bridge/ksp_bridge.hpp>
#include <ksp_bridge/utils.hpp>

void KSPBridge::next_stage_srv(
    const ksp_bridge_interfaces::srv::Activation::Request::SharedPtr,
    const ksp_bridge_interfaces::srv::Activation::Response::SharedPtr res)
{
    bool succeded = false;
    try {
        m_vessel->control().activate_next_stage();
        succeded = true;
    } catch (const std::exception& ex) {
        res->error = std::string(ex.what());
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", base_name(__FILE__), __LINE__, ex.what());
    }

    res->succeded = succeded;
}