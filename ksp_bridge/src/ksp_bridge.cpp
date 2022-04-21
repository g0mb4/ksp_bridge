#include <krpc/services/krpc.hpp>
#include <ksp_bridge/ksp_bridge.hpp>
#include <ksp_bridge/utils.hpp>

#include <stdlib.h>

KSPBridge::KSPBridge()
    : rclcpp::Node("ksp_bridge")
{
    declare_parameter<int64_t>("update_interval_ms", 100);
    declare_parameter<std::vector<std::string>>("celestial_bodies", { "sun", "kerbin", "mun" });

    int64_t update_interval_ms = get_parameter("update_interval_ms").as_int();
    m_param_celestial_bodies = get_parameter("celestial_bodies").as_string_array();

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

    if (scene == krpc::services::KRPC::GameScene::flight) {
        return true;
    } else {
        RCLCPP_WARN(get_logger(), "Invalid game screen.");
        invalidate_active_vessel();
        return false;
    }
}

void KSPBridge::validate_active_vessel()
{
    // vessel has been invalidated
    if (m_vessel == nullptr) {
        connect();
        find_active_vessel();
        return;
    }

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

void KSPBridge::invalidate_active_vessel()
{
    m_vessel = nullptr;
    RCLCPP_WARN(get_logger(), "Vessel has been invalidated.");
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
    auto bodies = m_space_center->bodies();

    for (auto it = bodies.begin(); it != bodies.end(); ++it) {
        auto name = it->second.name();
        auto name_lower = str_lowercase(name);

        auto it_find = std::find(m_param_celestial_bodies.begin(), m_param_celestial_bodies.end(), name_lower);

        if (it_find != m_param_celestial_bodies.end()) {
            m_celestial_bodies[name_lower] = it->second;
        }
    }

    auto it = m_celestial_bodies.find("sun");
    if (it == m_celestial_bodies.end()) {
        RCLCPP_FATAL(get_logger(), "Sun is a required celestial body.");
        exit(1);
    }

    if (!change_reference_frame("kerbin")) {
        RCLCPP_FATAL(get_logger(), "Unable to change the reference frame to kerbin.");
        exit(1);
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
    m_orbit_publisher = create_publisher<ksp_bridge_interfaces::msg::Orbit>("/vessel/orbit", 10);

    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    m_cmd_throttle_sub = create_subscription<ksp_bridge_interfaces::msg::CmdThrottle>(
        "/cmd_throttle",
        10,
        std::bind(&KSPBridge::cmd_throttle_sub, this, std::placeholders::_1));

    m_cmd_rotation_sub = create_subscription<ksp_bridge_interfaces::msg::CmdRotation>(
        "/cmd_rotation",
        10,
        std::bind(&KSPBridge::cmd_rotation_sub, this, std::placeholders::_1));

    m_next_stage_srv = create_service<ksp_bridge_interfaces::srv::Activation>(
        "/next_stage",
        std::bind(
            &KSPBridge::next_stage_srv, this,
            std::placeholders::_1, std::placeholders::_2));

    m_sas_srv = create_service<ksp_bridge_interfaces::srv::SAS>(
        "/sas",
        std::bind(
            &KSPBridge::sas_srv, this,
            std::placeholders::_1, std::placeholders::_2));
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

    if (gather_orbit_data()) {
        m_orbit_publisher->publish(m_orbit_data);
    }

    send_tf_tree();
}
