#include "ksp_bridge/ksp_bridge.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<KSPBridge>());

    rclcpp::shutdown();

    return 0;
}