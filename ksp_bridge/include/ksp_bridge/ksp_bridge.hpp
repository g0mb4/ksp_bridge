#pragma once

#include <krpc.hpp>
#include <krpc/services/krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <ksp_bridge_interfaces/msg/control_data.hpp>
#include <ksp_bridge_interfaces/msg/vessel_data.hpp>
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

    bool gather_data(ksp_bridge_interfaces::msg::VesselData& vessel, ksp_bridge_interfaces::msg::ControlData& control);

    void publish_data();

    std::unique_ptr<krpc::Client> m_ksp_client;
    std::unique_ptr<krpc::services::KRPC> m_krpc;
    std::unique_ptr<krpc::services::SpaceCenter> m_space_center;

    std::unique_ptr<krpc::services::SpaceCenter::Vessel> m_vessel;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::VesselData>::SharedPtr m_vessel_data_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::ControlData>::SharedPtr m_control_data_publisher;

    rclcpp::TimerBase::SharedPtr m_publish_timer;
};