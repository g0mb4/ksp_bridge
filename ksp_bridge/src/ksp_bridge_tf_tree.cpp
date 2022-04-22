#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ksp_bridge/ksp_bridge.hpp>
#include <ksp_bridge/utils.hpp>

bool KSPBridge::change_reference_frame(const std::string& name)
{
    m_refrence_frame.lock.lock();
    if (name == m_refrence_frame.name) {
        RCLCPP_WARN(get_logger(), "Reference frame is the same, will not be changed.");
        m_refrence_frame.lock.unlock();
        return true;
    }

    if (name == "vessel") {
        RCLCPP_INFO(get_logger(), "Reference frame is changed: '%s' -> '%s'", m_refrence_frame.name.c_str(), name.c_str());

        m_refrence_frame.name = "vessel";
        m_refrence_frame.refrence_frame = m_vessel->reference_frame();
        m_refrence_frame.lock.unlock();
        return true;
    }

    auto it = m_celestial_bodies.find(name);
    if (it != m_celestial_bodies.end()) {
        RCLCPP_INFO(get_logger(), "Reference frame is changed: '%s' -> '%s'", m_refrence_frame.name.c_str(), name.c_str());

        m_refrence_frame.name = name;
        m_refrence_frame.refrence_frame = it->second.reference_frame();
        m_refrence_frame.lock.unlock();
        return true;
    }

    RCLCPP_ERROR(get_logger(), "%s:%d: Unknown reference frame: '%s'", base_name(__FILE__), __LINE__, name.c_str());
    m_refrence_frame.lock.unlock();
    return false;
}

void KSPBridge::send_tf_tree(NamedReferenceFrame& frame)
{
    try {
        // world -> kerbin, world = kerbin
        // Note: Kerbin is always present.
        geometry_msgs::msg::TransformStamped t_wordl_kerbin;
        t_wordl_kerbin.header.stamp = now();
        t_wordl_kerbin.header.frame_id = "world";
        t_wordl_kerbin.child_frame_id = "kerbin";

        m_tf_broadcaster->sendTransform(t_wordl_kerbin);

        // kerbin -> bodies
        auto kerbin_rf = m_celestial_bodies["kerbin"].reference_frame();
        for (auto it = m_celestial_bodies.begin(); it != m_celestial_bodies.end(); ++it) {
            auto name = it->first;
            auto body = it->second;

            // kerbin -> kerbin transform is not required
            if (name == "kerbin") {
                continue;
            }

            auto body_rf = body.reference_frame();

            auto pos_kerbin = m_vessel->position(kerbin_rf);
            auto rot_kerbin = m_vessel->rotation(kerbin_rf);

            auto t_kerbin_body = get_transform(*m_space_center, pos_kerbin, rot_kerbin, kerbin_rf, body_rf);
            t_kerbin_body.header.stamp = now();
            t_kerbin_body.header.frame_id = "kerbin";
            t_kerbin_body.child_frame_id = name;
            m_tf_broadcaster->sendTransform(t_kerbin_body);
        }

        // vessel -> current reference frame
        std::string vessel_current = "kerbin";
        if (frame.name != "vessel") {
            vessel_current = frame.name;
        }
        auto vessel_rf = m_vessel->reference_frame();
        auto current_rf = m_celestial_bodies[vessel_current].reference_frame();

        auto pos_current = m_vessel->position(current_rf);
        auto rot_current = m_vessel->rotation(current_rf);

        auto t_vessel_current = get_transform(*m_space_center, pos_current, rot_current, current_rf, vessel_rf);
        t_vessel_current.header.stamp = now();
        t_vessel_current.header.frame_id = vessel_current;
        t_vessel_current.child_frame_id = "vessel";
        m_tf_broadcaster->sendTransform(t_vessel_current);

        // vessel -> parts
        for (auto& part : m_parts_data.parts) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now();
            t.header.frame_id = "vessel";
            t.child_frame_id = part.title + part.tag;

            t.transform.translation.x = part.position.x;
            t.transform.translation.y = part.position.y;
            t.transform.translation.z = part.position.z;

            t.transform.rotation.x = part.rotation.x;
            t.transform.rotation.y = part.rotation.y;
            t.transform.rotation.z = part.rotation.z;
            t.transform.rotation.w = part.rotation.w;

            m_tf_broadcaster->sendTransform(t);
        }

    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", base_name(__FILE__), __LINE__, ex.what());
        invalidate_active_vessel();
    }
}