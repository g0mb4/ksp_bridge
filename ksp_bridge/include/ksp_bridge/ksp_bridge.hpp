#pragma once

#include <krpc.hpp>
#include <krpc/services/krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <ksp_bridge_interfaces/msg/celestial_bodies.hpp>
#include <ksp_bridge_interfaces/msg/cmd_rotation.hpp>
#include <ksp_bridge_interfaces/msg/cmd_throttle.hpp>
#include <ksp_bridge_interfaces/msg/control.hpp>
#include <ksp_bridge_interfaces/msg/flight.hpp>
#include <ksp_bridge_interfaces/msg/orbit.hpp>
#include <ksp_bridge_interfaces/msg/parts.hpp>
#include <ksp_bridge_interfaces/msg/vessel.hpp>
#include <ksp_bridge_interfaces/srv/activation.hpp>
#include <ksp_bridge_interfaces/srv/sas.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class KSPBridge : public rclcpp::Node {
public:
    KSPBridge();

private:
    void connect();
    bool is_valid_screen();

    void validate_active_vessel();
    void invalidate_active_vessel();
    void find_active_vessel();
    void init_interfaces();
    void init_celestial_bodies();

    bool change_reference_frame(const std::string& name);

    struct NamedReferenceFrame {
        std::mutex lock;
        std::string name;
        krpc::services::SpaceCenter::ReferenceFrame refrence_frame;
    };

    std::unique_ptr<krpc::Client> m_ksp_client;
    std::unique_ptr<krpc::services::KRPC> m_krpc;
    std::unique_ptr<krpc::services::SpaceCenter> m_space_center;
    std::vector<std::string> m_param_celestial_bodies;
    std::unique_ptr<krpc::services::SpaceCenter::Vessel> m_vessel;
    NamedReferenceFrame m_refrence_frame;
    std::map<std::string, krpc::services::SpaceCenter::CelestialBody> m_celestial_bodies;

    // publishers
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Vessel>::SharedPtr m_vessel_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Control>::SharedPtr m_control_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Flight>::SharedPtr m_flight_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Parts>::SharedPtr m_parts_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::CelestialBodies>::SharedPtr m_celestial_bodies_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Orbit>::SharedPtr m_orbit_publisher;

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    rclcpp::TimerBase::SharedPtr m_publish_timer;

    ksp_bridge_interfaces::msg::Vessel m_vessel_data;
    ksp_bridge_interfaces::msg::Control m_control_data;
    ksp_bridge_interfaces::msg::Flight m_flight_data;
    ksp_bridge_interfaces::msg::Parts m_parts_data;
    ksp_bridge_interfaces::msg::CelestialBodies m_celestial_bodies_data;
    ksp_bridge_interfaces::msg::Orbit m_orbit_data;

    bool gather_vessel_data(NamedReferenceFrame& frame);
    bool gather_control_data(NamedReferenceFrame& frame);
    bool gather_flight_data(NamedReferenceFrame& frame);
    bool gather_parts_data();
    bool gather_celestial_bodies_data(NamedReferenceFrame& frame);
    bool gather_orbit_data();

    void send_tf_tree(NamedReferenceFrame& frame);

    void publish_data();

    // subscribers
    rclcpp::Subscription<ksp_bridge_interfaces::msg::CmdThrottle>::SharedPtr m_cmd_throttle_sub;
    rclcpp::Subscription<ksp_bridge_interfaces::msg::CmdRotation>::SharedPtr m_cmd_rotation_sub;

    void cmd_throttle_sub(const ksp_bridge_interfaces::msg::CmdThrottle::SharedPtr);
    void cmd_rotation_sub(const ksp_bridge_interfaces::msg::CmdRotation::SharedPtr);

    // servers
    rclcpp::Service<ksp_bridge_interfaces::srv::Activation>::SharedPtr m_next_stage_srv;
    rclcpp::Service<ksp_bridge_interfaces::srv::SAS>::SharedPtr m_sas_srv;

    void next_stage_srv(
        const ksp_bridge_interfaces::srv::Activation::Request::SharedPtr,
        const ksp_bridge_interfaces::srv::Activation::Response::SharedPtr);

    void sas_srv(
        const ksp_bridge_interfaces::srv::SAS::Request::SharedPtr,
        const ksp_bridge_interfaces::srv::SAS::Response::SharedPtr);
};