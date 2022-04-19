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
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
    }

    init_communication();
}

void KSPBridge::init_communication()
{
    m_vessel_publisher = create_publisher<ksp_bridge_interfaces::msg::Vessel>("/vessel", 10);
    m_control_publisher = create_publisher<ksp_bridge_interfaces::msg::Control>("/control", 10);
    m_flight_publisher = create_publisher<ksp_bridge_interfaces::msg::Flight>("/flight", 10);

    m_throttle_sub = create_subscription<ksp_bridge_interfaces::msg::Float>(
        "/throttle",
        10,
        std::bind(&KSPBridge::throttle_sub, this, std::placeholders::_1));
}

bool KSPBridge::gather_data()
{
    if (!is_valid_screen()) {
        return false;
    }

    try {
        auto flight = m_vessel->flight();
        auto control = m_vessel->control();

        m_vessel_data.header.frame_id = m_refrence_frame.name;
        m_vessel_data.header.stamp = now();
        m_vessel_data.name = m_vessel->name();
        m_vessel_data.type = (uint8_t)m_vessel->type();
        m_vessel_data.situation = (uint8_t)m_vessel->situation();
        m_vessel_data.recoverable = m_vessel->recoverable();

        m_vessel_data.met = m_vessel->met();
        m_vessel_data.biome = m_vessel->biome();
        m_vessel_data.crew_capacity = m_vessel->crew_capacity();
        m_vessel_data.crew_count = m_vessel->crew_count();
        m_vessel_data.mass = m_vessel->mass();
        m_vessel_data.dry_mass = m_vessel->dry_mass();

        m_vessel_data.thrust = m_vessel->thrust();
        m_vessel_data.available_thrust = m_vessel->available_thrust();
        m_vessel_data.max_thrust = m_vessel->max_thrust();
        m_vessel_data.max_vacuum_thrust = m_vessel->max_vacuum_thrust();
        m_vessel_data.specific_impulse = m_vessel->specific_impulse();
        m_vessel_data.vacuum_specific_impulse = m_vessel->vacuum_specific_impulse();
        m_vessel_data.kerbin_sea_level_specific_impulse = m_vessel->kerbin_sea_level_specific_impulse();

        m_vessel_data.moment_of_inertia = tuple2vector3(m_vessel->moment_of_inertia());

        m_vessel_data.inertia.m = m_vessel->mass();
        m_vessel_data.inertia.com = tuple2vector3(m_vessel->position(m_refrence_frame.refrence_frame));
        // TODO: check this
        m_vessel_data.inertia.ixx = m_vessel->inertia_tensor()[0];
        m_vessel_data.inertia.ixy = m_vessel->inertia_tensor()[1];
        m_vessel_data.inertia.ixz = m_vessel->inertia_tensor()[2];
        m_vessel_data.inertia.iyy = m_vessel->inertia_tensor()[3];
        m_vessel_data.inertia.iyz = m_vessel->inertia_tensor()[4];
        m_vessel_data.inertia.izz = m_vessel->inertia_tensor()[5];

        m_vessel_data.position = tuple2vector3(m_vessel->position(m_refrence_frame.refrence_frame));
        m_vessel_data.velocity = tuple2vector3(m_vessel->velocity(m_refrence_frame.refrence_frame));
        m_vessel_data.rotation = tuple2quaternion(m_vessel->rotation(m_refrence_frame.refrence_frame));
        m_vessel_data.direction = tuple2vector3(m_vessel->direction(m_refrence_frame.refrence_frame));
        m_vessel_data.angular_velocity = tuple2vector3(m_vessel->angular_velocity(m_refrence_frame.refrence_frame));

        m_flight_data.g_force = flight.g_force();
        m_flight_data.mean_altitude = flight.mean_altitude();
        m_flight_data.surface_altitude = flight.surface_altitude();
        m_flight_data.bedrock_altitude = flight.bedrock_altitude();
        m_flight_data.elevation = flight.elevation();
        m_flight_data.latitude = flight.latitude();
        m_flight_data.longitude = flight.longitude();

        m_control_data.header.frame_id = m_refrence_frame.name;
        m_control_data.header.stamp = now();
        m_control_data.source = (uint8_t)control.source();
        m_control_data.state = (uint8_t)control.state();
        m_control_data.sas = control.sas();
        m_control_data.sas_mode = (uint8_t)control.sas_mode();
        m_control_data.speed_mode = (uint8_t)control.speed_mode();
        m_control_data.rcs = control.rcs();
        m_control_data.reaction_wheels = control.reaction_wheels();
        m_control_data.gear = control.gear();
        m_control_data.legs = control.legs();
        m_control_data.wheels = control.wheels();
        m_control_data.lights = control.lights();
        m_control_data.brakes = control.brakes();
        m_control_data.antennas = control.antennas();
        m_control_data.cargo_bays = control.cargo_bays();
        m_control_data.intakes = control.intakes();
        m_control_data.parachutes = control.parachutes();
        m_control_data.radiators = control.radiators();
        m_control_data.resource_harvesters = control.resource_harvesters();
        m_control_data.resource_harvesters_active = control.resource_harvesters_active();
        m_control_data.solar_panels = control.solar_panels();
        m_control_data.abort = control.abort();
        m_control_data.throttle = control.throttle();
        m_control_data.input_mode = (uint8_t)control.input_mode();
        m_control_data.pitch = control.pitch();
        m_control_data.yaw = control.yaw();
        m_control_data.roll = control.roll();
        m_control_data.forward = control.forward();
        m_control_data.up = control.up();
        m_control_data.right = control.right();
        m_control_data.wheel_throttle = control.wheel_throttle();
        m_control_data.wheel_steering = control.wheel_steering();
        m_control_data.current_stage = control.current_stage();
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
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

void KSPBridge::throttle_sub(const ksp_bridge_interfaces::msg::Float::SharedPtr msg)
{
    if (msg->value > 1 || msg->value < 0) {
        RCLCPP_WARN(get_logger(), "Throttle must be between 0 and 1, current value is %f, will be clamped.", msg->value);
    }

    float value = clamp<float>(msg->value, 0, 1);

    try {
        m_vessel->control().set_throttle(value);
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
    }
}