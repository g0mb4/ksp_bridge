#include "ksp_bridge/ksp_bridge.hpp"
#include "ksp_bridge/utils.hpp"
#include "ksp_bridge_interfaces/msg/resource.hpp"

#include <krpc/services/krpc.hpp>

bool KSPBridge::gather_vessel_data()
{
    try {
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
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", __FILE__, __LINE__, ex.what());
        return false;
    }

    return true;
}

bool KSPBridge::gather_control_data()
{
    try {
        auto control = m_vessel->control();

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
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", __FILE__, __LINE__, ex.what());
        return false;
    }

    return true;
}

bool KSPBridge::gather_flight_data()
{
    try {
        auto flight = m_vessel->flight(m_refrence_frame.refrence_frame);

        m_flight_data.header.frame_id = m_refrence_frame.name;
        m_flight_data.header.stamp = now();

        m_flight_data.g_force = flight.g_force();
        m_flight_data.mean_altitude = flight.mean_altitude();
        m_flight_data.surface_altitude = flight.surface_altitude();
        m_flight_data.bedrock_altitude = flight.bedrock_altitude();
        m_flight_data.velocity = tuple2vector3(flight.velocity());
        m_flight_data.speed = flight.speed();
        m_flight_data.horizontal_speed = flight.horizontal_speed();
        m_flight_data.vertical_speed = flight.vertical_speed();
        m_flight_data.center_of_mass = tuple2vector3(flight.center_of_mass());
        m_flight_data.rotation = tuple2quaternion(flight.rotation());
        m_flight_data.direction = tuple2vector3(flight.direction());
        m_flight_data.pitch = flight.pitch();
        m_flight_data.heading = flight.heading();
        m_flight_data.roll = flight.roll();
        m_flight_data.prograde = tuple2vector3(flight.prograde());
        m_flight_data.retrograde = tuple2vector3(flight.retrograde());
        m_flight_data.normal = tuple2vector3(flight.normal());
        m_flight_data.anti_normal = tuple2vector3(flight.anti_normal());
        m_flight_data.radial = tuple2vector3(flight.radial());
        m_flight_data.anti_radial = tuple2vector3(flight.anti_radial());
        m_flight_data.atmosphere_density = flight.atmosphere_density();
        m_flight_data.dynamic_pressure = flight.dynamic_pressure();
        m_flight_data.static_pressure = flight.static_pressure();
        m_flight_data.static_pressure_at_msl = flight.static_pressure_at_msl();
        m_flight_data.aerodynamic_force = tuple2vector3(flight.aerodynamic_force());
        m_flight_data.lift = tuple2vector3(flight.lift());
        m_flight_data.drag = tuple2vector3(flight.drag());
        m_flight_data.speed_of_sound = flight.speed_of_sound();
        m_flight_data.mach = flight.mach();
        m_flight_data.true_air_speed = flight.true_air_speed();
        m_flight_data.equivalent_air_speed = flight.equivalent_air_speed();
        m_flight_data.terminal_velocity = flight.terminal_velocity();
        m_flight_data.angle_of_attack = flight.angle_of_attack();
        m_flight_data.sideslip_angle = flight.sideslip_angle();
        m_flight_data.total_air_temperature = flight.total_air_temperature();
        m_flight_data.static_air_temperature = flight.static_air_temperature();
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", __FILE__, __LINE__, ex.what());
        return false;
    }

    return true;
}

bool KSPBridge::gather_parts_data()
{
    try {
        auto parts = m_vessel->parts().all();
        m_parts_data.parts.clear();

        m_parts_data.header.frame_id = "vessel";
        m_parts_data.header.stamp = now();
        auto vessel_rf = m_vessel->reference_frame();

        for (auto& part : parts) {
            try {
                auto part_data = ksp_bridge_interfaces::msg::Part();

                part_data.name = part.name();
                part_data.title = part.title();
                //part_data.tag = part.tag();
                part_data.highlighted = part.highlighted();
                part_data.highlight_color = tuple2vector3(part.highlight_color());
                part_data.cost = part.cost();
                part_data.axially_attached = part.axially_attached();
                part_data.radially_attached = part.radially_attached();
                part_data.stage = part.stage();
                part_data.decouple_stage = part.decouple_stage();
                part_data.massless = part.massless();
                part_data.mass = part.mass();
                part_data.dry_mass = part.dry_mass();
                part_data.shielded = part.shielded();
                part_data.dynamic_pressure = part.dynamic_pressure();
                part_data.impact_tolerance = part.impact_tolerance();
                part_data.temperature = part.temperature();
                part_data.skin_temperature = part.skin_temperature();
                part_data.max_temperature = part.max_temperature();
                part_data.max_skin_temperature = part.max_skin_temperature();
                part_data.thermal_mass = part.thermal_mass();
                part_data.thermal_skin_mass = part.thermal_skin_mass();
                part_data.thermal_resource_mass = part.thermal_resource_mass();
                part_data.thermal_conduction_flux = part.thermal_conduction_flux();
                part_data.thermal_convection_flux = part.thermal_convection_flux();
                part_data.thermal_radiation_flux = part.thermal_radiation_flux();
                part_data.thermal_internal_flux = part.thermal_internal_flux();
                part_data.thermal_skin_to_internal_flux = part.thermal_skin_to_internal_flux();

                auto resources = part.resources().all();
                for (auto& resource : resources) {
                    ksp_bridge_interfaces::msg::Resource r;

                    r.name = resource.name();
                    r.amount = resource.amount();
                    r.max = resource.max();
                    r.density = resource.density();
                    r.flow_mode = (uint8_t)resource.flow_mode();
                    r.enabled = resource.enabled();

                    part_data.resources.emplace_back(r);
                }

                part_data.crossfeed = part.crossfeed();
                part_data.is_fuel_line = part.is_fuel_line();
                // TODO: reference frame of the vessel ???
                part_data.position = tuple2vector3(part.position(vessel_rf));
                part_data.center_of_mass = tuple2vector3(part.center_of_mass(vessel_rf));
                part_data.direction = tuple2vector3(part.direction(vessel_rf));
                part_data.velocity = tuple2vector3(part.velocity(vessel_rf));
                part_data.rotation = tuple2quaternion(part.rotation(vessel_rf));
                part_data.moment_of_inertia = tuple2vector3(part.moment_of_inertia());

                part_data.inertia.m = part.mass();
                part_data.inertia.com = tuple2vector3(part.position(vessel_rf));
                // TODO: check this
                part_data.inertia.ixx = part.inertia_tensor()[0];
                part_data.inertia.ixy = part.inertia_tensor()[1];
                part_data.inertia.ixz = part.inertia_tensor()[2];
                part_data.inertia.iyy = part.inertia_tensor()[3];
                part_data.inertia.iyz = part.inertia_tensor()[4];
                part_data.inertia.izz = part.inertia_tensor()[5];

                m_parts_data.parts.emplace_back(part_data);
            } catch (const std::exception& ex) {
                RCLCPP_ERROR(get_logger(), "%s:%d: %s", __FILE__, __LINE__, ex.what());
                continue;
            }
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", __FILE__, __LINE__, ex.what());
        return false;
    }

    return true;
}
