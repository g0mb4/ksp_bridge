#pragma once

#include <krpc.hpp>
#include <krpc/services/krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <ksp_bridge_interfaces/msg/control.hpp>
#include <ksp_bridge_interfaces/msg/flight.hpp>
#include <ksp_bridge_interfaces/msg/vessel.hpp>
#include <rclcpp/rclcpp.hpp>

class KSPBridge : public rclcpp::Node {
public:
    KSPBridge();

private:
    void connect();
    void validate_active_vessel();
    bool is_valid_screen();

    void find_active_vessel();
    void init_publishers();

    bool gather_data();

    void publish_data();

    std::unique_ptr<krpc::Client> m_ksp_client;
    std::unique_ptr<krpc::services::KRPC> m_krpc;
    std::unique_ptr<krpc::services::SpaceCenter> m_space_center;

    std::unique_ptr<krpc::services::SpaceCenter::Vessel> m_vessel;

    rclcpp::Publisher<ksp_bridge_interfaces::msg::Vessel>::SharedPtr m_vessel_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Control>::SharedPtr m_control_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Flight>::SharedPtr m_flight_publisher;

    rclcpp::TimerBase::SharedPtr m_publish_timer;

    ksp_bridge_interfaces::msg::Vessel m_vessel_data;
    ksp_bridge_interfaces::msg::Control m_control_data;
    ksp_bridge_interfaces::msg::Flight m_flight_data;
};