#include "ksp_bridge/ksp_bridge.hpp"
#include "ksp_bridge/utils.hpp"

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
    m_celestial_bodies = m_space_center->bodies();

    try {
        m_refrence_frame.name = "kerbin";
        m_refrence_frame.refrence_frame = m_celestial_bodies["Kerbin"].reference_frame();
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", __FILE__, __LINE__, ex.what());
    }

    init_communication();
}

void KSPBridge::init_communication()
{
    m_vessel_publisher = create_publisher<ksp_bridge_interfaces::msg::Vessel>("/vessel", 10);
    m_control_publisher = create_publisher<ksp_bridge_interfaces::msg::Control>("/vessel/control", 10);
    m_flight_publisher = create_publisher<ksp_bridge_interfaces::msg::Flight>("/vessel/flight", 10);
    m_parts_publisher = create_publisher<ksp_bridge_interfaces::msg::Parts>("/vessel/parts", 10);
    m_celestial_bodies_publisher = create_publisher<ksp_bridge_interfaces::msg::CelestialBodies>("/celestial_bodies", 10);

    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    m_throttle_sub = create_subscription<ksp_bridge_interfaces::msg::Float>(
        "/throttle",
        10,
        std::bind(&KSPBridge::throttle_sub, this, std::placeholders::_1));
}

void KSPBridge::publish_data()
{
    if (!is_valid_screen()) {
        return;
    }

    validate_active_vessel();

    if (gather_vessel_data()) {
        m_vessel_publisher->publish(m_vessel_data);
    }

    if (gather_control_data()) {
        m_control_publisher->publish(m_control_data);
    }

    if (gather_flight_data()) {
        m_flight_publisher->publish(m_flight_data);
    }

    if (gather_parts_data()) {
        m_parts_publisher->publish(m_parts_data);
    }

    if (gather_celestial_bodies_data()) {
        m_celestial_bodies_publisher->publish(m_celestial_bodies_data);
    }

    send_tf_tree();
}
