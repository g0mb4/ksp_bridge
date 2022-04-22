#include <ksp_bridge/ksp_bridge.hpp>
#include <ksp_bridge/utils.hpp>

void KSPBridge::next_stage_srv(
    const ksp_bridge_interfaces::srv::Activation::Request::SharedPtr,
    const ksp_bridge_interfaces::srv::Activation::Response::SharedPtr res)
{
    try {
        m_vessel->control().activate_next_stage();
    } catch (const std::exception& ex) {
        res->succeded = false;
        res->error = std::string(ex.what());
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", base_name(__FILE__), __LINE__, ex.what());
    }

    res->succeded = true;
}

void KSPBridge::set_sas_srv(
    const ksp_bridge_interfaces::srv::SAS::Request::SharedPtr req,
    const ksp_bridge_interfaces::srv::SAS::Response::SharedPtr res)
{
    krpc::services::SpaceCenter::SASMode mode = krpc::services::SpaceCenter::SASMode::stability_assist;

    switch (req->mode) {
    case ksp_bridge_interfaces::srv::SAS::Request::SAS_MODE_STABILITY_ASSIST:
        mode = krpc::services::SpaceCenter::SASMode::stability_assist;
        break;

    case ksp_bridge_interfaces::srv::SAS::Request::SAS_MODE_MANEUVER:
        mode = krpc::services::SpaceCenter::SASMode::maneuver;
        break;

    case ksp_bridge_interfaces::srv::SAS::Request::SAS_MODE_PROGRADE:
        mode = krpc::services::SpaceCenter::SASMode::prograde;
        break;

    case ksp_bridge_interfaces::srv::SAS::Request::SAS_MODE_RETROGRADE:
        mode = krpc::services::SpaceCenter::SASMode::retrograde;
        break;

    case ksp_bridge_interfaces::srv::SAS::Request::SAS_MODE_NORMAL:
        mode = krpc::services::SpaceCenter::SASMode::normal;
        break;

    case ksp_bridge_interfaces::srv::SAS::Request::SAS_MODE_ANTI_NORMAL:
        mode = krpc::services::SpaceCenter::SASMode::anti_normal;
        break;

    case ksp_bridge_interfaces::srv::SAS::Request::SAS_MODE_RADIAL:
        mode = krpc::services::SpaceCenter::SASMode::radial;
        break;

    case ksp_bridge_interfaces::srv::SAS::Request::SAS_MODE_ANTI_RADIAL:
        mode = krpc::services::SpaceCenter::SASMode::anti_radial;
        break;

    case ksp_bridge_interfaces::srv::SAS::Request::SAS_MODE_TARGET:
        mode = krpc::services::SpaceCenter::SASMode::target;
        break;

    case ksp_bridge_interfaces::srv::SAS::Request::SAS_MODE_ANTI_TARGET:
        mode = krpc::services::SpaceCenter::SASMode::anti_target;
        break;

    default:
        res->succeded = false;
        res->error = "Unknown SAS mode.";
        RCLCPP_ERROR(get_logger(), "%s:%d: Unknown SAS mode.", base_name(__FILE__), __LINE__);
        return;
    }

    try {
        m_vessel->control().set_sas_mode(mode);
    } catch (const std::exception& ex) {
        res->succeded = false;
        res->error = std::string(ex.what());
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", base_name(__FILE__), __LINE__, ex.what());
        return;
    }

    try {
        m_vessel->control().set_sas(req->enabled);
    } catch (const std::exception& ex) {
        res->succeded = false;
        res->error = std::string(ex.what());
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", base_name(__FILE__), __LINE__, ex.what());
        return;
    }

    res->succeded = true;
}

void KSPBridge::set_reference_frame(
    const ksp_bridge_interfaces::srv::String::Request::SharedPtr req,
    const ksp_bridge_interfaces::srv::String::Response::SharedPtr res)
{
    bool success = change_reference_frame(req->value);

    res->succeded = success;
    if (!success) {
        res->error = "Unknown reference frame";
        RCLCPP_ERROR(get_logger(), "%s:%d: Unknown reference frame: '%s'", base_name(__FILE__), __LINE__, req->value.c_str());
    }
}