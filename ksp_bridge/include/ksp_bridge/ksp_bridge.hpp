#pragma once

#include <krpc.hpp>
#include <krpc/services/krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <ksp_bridge_interfaces/msg/celestial_bodies.hpp>
#include <ksp_bridge_interfaces/msg/control.hpp>
#include <ksp_bridge_interfaces/msg/flight.hpp>
#include <ksp_bridge_interfaces/msg/float.hpp>
#include <ksp_bridge_interfaces/msg/parts.hpp>
#include <ksp_bridge_interfaces/msg/vessel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class KSPBridge : public rclcpp::Node {
public:
    KSPBridge();

private:
    void connect();
    void validate_active_vessel();
    bool is_valid_screen();

    void find_active_vessel();
    void init_communication();

    bool gather_vessel_data();
    bool gather_control_data();
    bool gather_flight_data();
    bool gather_parts_data();
    bool gather_celestial_bodies_data();

    void send_tf_tree();

    void publish_data();

    struct NamedReferenceFrame {
        std::string name;
        krpc::services::SpaceCenter::ReferenceFrame refrence_frame;
    };

    void throttle_sub(const ksp_bridge_interfaces::msg::Float::SharedPtr msg);

    std::unique_ptr<krpc::Client> m_ksp_client;
    std::unique_ptr<krpc::services::KRPC> m_krpc;
    std::unique_ptr<krpc::services::SpaceCenter> m_space_center;

    std::unique_ptr<krpc::services::SpaceCenter::Vessel> m_vessel;
    NamedReferenceFrame m_refrence_frame;

    std::map<std::string, krpc::services::SpaceCenter::CelestialBody> m_celestial_bodies;

    rclcpp::Publisher<ksp_bridge_interfaces::msg::Vessel>::SharedPtr m_vessel_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Control>::SharedPtr m_control_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Flight>::SharedPtr m_flight_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Parts>::SharedPtr m_parts_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::CelestialBodies>::SharedPtr m_celestial_bodies_publisher;

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    rclcpp::TimerBase::SharedPtr m_publish_timer;

    ksp_bridge_interfaces::msg::Vessel m_vessel_data;
    ksp_bridge_interfaces::msg::Control m_control_data;
    ksp_bridge_interfaces::msg::Flight m_flight_data;
    ksp_bridge_interfaces::msg::Parts m_parts_data;
    ksp_bridge_interfaces::msg::CelestialBodies m_celestial_bodies_data;

    rclcpp::Subscription<ksp_bridge_interfaces::msg::Float>::SharedPtr m_throttle_sub;
};