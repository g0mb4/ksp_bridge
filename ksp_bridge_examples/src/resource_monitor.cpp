#include <ksp_bridge_interfaces/msg/parts.hpp>
#include <rclcpp/rclcpp.hpp>

class KSPBExampleResourceMonitor : public rclcpp::Node {
public:
    KSPBExampleResourceMonitor()
        : Node("kspb_example_resource_monitor")
    {
        m_parts_sub = create_subscription<ksp_bridge_interfaces::msg::Parts>(
            "/vessel/parts",
            10,
            [this](const ksp_bridge_interfaces::msg::Parts::SharedPtr msg) {
                auto parts = msg->parts;

                m_resources.clear();
                for (const auto& part : parts) {
                    auto resources = part.resources;

                    for (const auto& resource : resources) {
                        m_resources[resource.name] += resource.amount;
                    }
                }

                RCLCPP_INFO(get_logger(), "------ Available resources:");
                for (const auto& r : m_resources) {
                    RCLCPP_INFO(get_logger(), "%s : %f", r.first.c_str(), r.second);
                }
            });
    }

private:
    rclcpp::Subscription<ksp_bridge_interfaces::msg::Parts>::SharedPtr m_parts_sub;
    std::map<std::string, double> m_resources;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KSPBExampleResourceMonitor>());
    rclcpp::shutdown();
    return 0;
}