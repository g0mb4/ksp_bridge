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
    m_vessel_publisher = create_publisher<ksp_bridge_interfaces::msg::Vessel>("/vessel", 10);
    m_control_publisher = create_publisher<ksp_bridge_interfaces::msg::Control>("/control", 10);
    m_flight_publisher = create_publisher<ksp_bridge_interfaces::msg::Flight>("/flight", 10);
}

bool KSPBridge::gather_data()
{
    if (!is_valid_screen()) {
        return false;
    }

    try {
        auto flight = m_vessel->flight();
        auto control = m_vessel->control();

        m_vessel_data.name = m_vessel->name();
        m_vessel_data.type = (uint8_t)m_vessel->type();
        m_vessel_data.mass = m_vessel->mass();
        m_vessel_data.dry_mass = m_vessel->dry_mass();

        m_flight_data.g_force = flight.g_force();
        m_flight_data.mean_altitude = flight.mean_altitude();
        m_flight_data.surface_altitude = flight.surface_altitude();
        m_flight_data.bedrock_altitude = flight.bedrock_altitude();
        m_flight_data.elevation = flight.elevation();
        m_flight_data.latitude = flight.latitude();
        m_flight_data.longitude = flight.longitude();

        m_control_data.sas = control.sas();
        m_control_data.rcs = control.rcs();
        m_control_data.gear = control.gear();
        m_control_data.legs = control.legs();
        m_control_data.wheels = control.wheels();
        m_control_data.lights = control.lights();
        m_control_data.brakes = control.brakes();
        m_control_data.throttle = control.throttle();
        m_control_data.pitch = control.pitch();
        m_control_data.yaw = control.yaw();
        m_control_data.roll = control.roll();
        m_control_data.forward = control.forward();
        m_control_data.up = control.up();
        m_control_data.right = control.right();
    } catch (...) {
        return false;
    }

    return true;
}

void KSPBridge::publish_data()
{
    validate_active_vessel();

    if (gather_data()) {
        m_vessel_publisher->publish(m_vessel_data);
        m_control_publisher->publish(m_control_data);
        m_flight_publisher->publish(m_flight_data);
    }
}