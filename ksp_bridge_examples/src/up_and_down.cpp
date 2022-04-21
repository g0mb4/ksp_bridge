#include <ksp_bridge_interfaces/msg/cmd_throttle.hpp>
#include <ksp_bridge_interfaces/msg/control.hpp>
#include <ksp_bridge_interfaces/msg/flight.hpp>
#include <ksp_bridge_interfaces/srv/activation.hpp>
#include <rclcpp/rclcpp.hpp>

class KSPExampleUpAndDown : public rclcpp::Node {
public:
    KSPExampleUpAndDown()
        : Node("kspb_example_up_and_down")
    {
        m_flight_sub = create_subscription<ksp_bridge_interfaces::msg::Flight>(
            "/vessel/flight",
            10,
            [this](const ksp_bridge_interfaces::msg::Flight::SharedPtr msg) {
                m_altitude = msg->mean_altitude;
                m_vertical_speed = msg->vertical_speed;
            });

        m_control_sub = create_subscription<ksp_bridge_interfaces::msg::Control>(
            "/vessel/control",
            10,
            [this](const ksp_bridge_interfaces::msg::Control::SharedPtr msg) {
                m_current_stage = msg->current_stage;
            });

        m_cmd_throttle_pub = create_publisher<ksp_bridge_interfaces::msg::CmdThrottle>(
            "/cmd_throttle", 10);

        RCLCPP_INFO(get_logger(), "launch in 3 ...");
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(get_logger(), "launch in 2 ...");
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(get_logger(), "launch in 1 ...");
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(get_logger(), "launch!");

        next_stage();
        m_update_timer = create_wall_timer(std::chrono::milliseconds(33),
            std::bind(&KSPExampleUpAndDown::update, this));
    }

private:
    static constexpr double THROTTLE_ALTITUDE = 1'000.0;
    static constexpr double PARACHUTE_SPEED = 100.0;

    void update()
    {
        if (m_current_stage == 2) {
            auto msg = ksp_bridge_interfaces::msg::CmdThrottle();

            if (m_altitude < THROTTLE_ALTITUDE) {
                msg.throttle = 1.0;
                m_cmd_throttle_pub->publish(msg);
            } else {
                msg.throttle = 0.0;
                m_cmd_throttle_pub->publish(msg);
                next_stage();
            }
        } else if (m_current_stage == 1) {
            if (m_vertical_speed < PARACHUTE_SPEED) {
                next_stage();
            }
        }
    }

    void next_stage()
    {
        auto thread = std::make_unique<std::thread>(
            [this] {
                next_stage_clnt();
            });

        thread->detach();
    }

    void next_stage_clnt()
    {
        auto client = create_client<ksp_bridge_interfaces::srv::Activation>("/next_stage");

        while (client->wait_for_service(std::chrono::seconds(1)) == false) {
            RCLCPP_WARN(get_logger(), "waiting for server ...");
        }

        auto request = std::make_shared<ksp_bridge_interfaces::srv::Activation::Request>();

        auto future = client->async_send_request(request);
        try {
            auto response = future.get();

            if (!response->succeded) {
                RCLCPP_ERROR(get_logger(), "staging error: %s", response->error.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "staging error: %s", e.what());
        }
    }

    rclcpp::TimerBase::SharedPtr m_update_timer;
    rclcpp::Subscription<ksp_bridge_interfaces::msg::Flight>::SharedPtr m_flight_sub;
    rclcpp::Subscription<ksp_bridge_interfaces::msg::Control>::SharedPtr m_control_sub;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::CmdThrottle>::SharedPtr m_cmd_throttle_pub;
    double m_altitude, m_vertical_speed;
    int m_current_stage;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KSPExampleUpAndDown>());
    rclcpp::shutdown();
    return 0;
}