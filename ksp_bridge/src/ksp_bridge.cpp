#include "ksp_bridge/ksp_bridge.hpp"

#include <krpc/services/krpc.hpp>

KSPBridge::KSPBridge()
    : rclcpp::Node("ksp_bridge")
{
    declare_parameter<int64_t>("update_interval_ms", 100);
    int64_t update_interval_ms = get_parameter("update_interval_ms").as_int();

    connect();
    find_active_vessel();

    m_publish_timer = create_wall_timer(std::chrono::milliseconds(update_interval_ms),
        std::bind(&KSPBridge::publish_data, this));
}

void KSPBridge::connect()
{
    while (rclcpp::ok()) {
        try {
            m_ksp_client = std::make_unique<krpc::Client>(krpc::connect("ksp_bridge"));

            m_space_center = std::make_unique<krpc::services::SpaceCenter>(m_ksp_client.get());
            m_krpc = std::make_unique<krpc::services::KRPC>(m_ksp_client.get());
            break;
        } catch (...) {
            RCLCPP_INFO(get_logger(), "Connecting to kRPC server ...");
        }

        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(get_logger(), "Connected to kRPC server v%s", m_krpc->get_status().version().c_str());
}

bool KSPBridge::is_valid_screen()
{
    auto scene = m_krpc->current_game_scene();
    if (scene == krpc::services::KRPC::GameScene::editor_sph
        || scene == krpc::services::KRPC::GameScene::editor_vab) {

        return false;
    }

    return true;
}

void KSPBridge::validate_active_vessel()
{
    bool is_valid = false;

    try {
        m_vessel = std::make_unique<krpc::services::SpaceCenter::Vessel>(m_space_center->active_vessel());
        is_valid = true;
    } catch (...) {
    }

    if (!is_valid) {
        find_active_vessel();
    }
}

void KSPBridge::find_active_vessel()
{
    while (rclcpp::ok()) {
        try {
            m_vessel = std::make_unique<krpc::services::SpaceCenter::Vessel>(m_space_center->active_vessel());
            break;
        } catch (...) {
            RCLCPP_INFO(get_logger(), "Searching active vessel ...");
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(get_logger(), "Vessel found: %s", m_vessel->name().c_str());
    init_publishers();
}

void KSPBridge::init_publishers()
{
    m_vessel_data_publisher = create_publisher<ksp_bridge_interfaces::msg::VesselData>("/vessel", 10);
    m_control_data_publisher = create_publisher<ksp_bridge_interfaces::msg::ControlData>("/control", 10);
}

bool KSPBridge::gather_data(ksp_bridge_interfaces::msg::VesselData& vessel_data,
    ksp_bridge_interfaces::msg::ControlData& control_data)
{
    if (!is_valid_screen()) {
        return false;
    }

    try {
        auto flight = m_vessel->flight();
        auto control = m_vessel->control();

        vessel_data.name = m_vessel->name();
        vessel_data.mass = m_vessel->mass();
        vessel_data.dry_mass = m_vessel->dry_mass();

        vessel_data.g_force = flight.g_force();
        vessel_data.mean_altitude = flight.mean_altitude();
        vessel_data.surface_altitude = flight.surface_altitude();
        vessel_data.bedrock_altitude = flight.bedrock_altitude();
        vessel_data.elevation = flight.elevation();
        vessel_data.latitude = flight.latitude();
        vessel_data.longitude = flight.longitude();

        control_data.sas = control.sas();
        control_data.rcs = control.rcs();
        control_data.gear = control.gear();
        control_data.legs = control.legs();
        control_data.wheels = control.wheels();
        control_data.lights = control.lights();
        control_data.brakes = control.brakes();
        control_data.throttle = control.throttle();
        control_data.pitch = control.pitch();
        control_data.yaw = control.yaw();
        control_data.roll = control.roll();
        control_data.forward = control.forward();
        control_data.up = control.up();
        control_data.right = control.right();
    } catch (...) {
        return false;
    }

    return true;
}

void KSPBridge::publish_data()
{
    validate_active_vessel();

    auto vessel_data = ksp_bridge_interfaces::msg::VesselData();
    auto control_data = ksp_bridge_interfaces::msg::ControlData();

    if (gather_data(vessel_data, control_data)) {
        m_vessel_data_publisher->publish(vessel_data);
        m_control_data_publisher->publish(control_data);
    }
}