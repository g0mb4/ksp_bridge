#include <algorithm>
#include <cctype>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ksp_bridge/ksp_bridge.hpp>
#include <ksp_bridge/utils.hpp>
#include <string>

void KSPBridge::send_tf_tree()
{
    // world -> sun, world = sun
    geometry_msgs::msg::TransformStamped t_ws;
    t_ws.header.stamp = now();
    t_ws.header.frame_id = "world";
    t_ws.child_frame_id = "sun";

    m_tf_broadcaster->sendTransform(t_ws);

    // sun -> bodies, bodies -> vessel
    auto sun_rf = m_celestial_bodies["Sun"].reference_frame();
    auto vessel_rf = m_vessel->reference_frame();
    for (auto it = m_celestial_bodies.begin(); it != m_celestial_bodies.end(); ++it) {
        if (it->first == "Sun") {
            continue;
        }

        auto body_name_to_lower = it->first;
        std::transform(body_name_to_lower.begin(), body_name_to_lower.end(), body_name_to_lower.begin(),
            [](unsigned char c) { return std::tolower(c); });

        auto body_rf = it->second.reference_frame();

        auto pos_sun = m_vessel->position(sun_rf);
        auto rot_sun = m_vessel->rotation(sun_rf);

        auto t_sb = get_transform(*m_space_center, pos_sun, rot_sun, sun_rf, body_rf);
        t_sb.header.stamp = now();
        t_sb.header.frame_id = "sun";
        t_sb.child_frame_id = body_name_to_lower;
        m_tf_broadcaster->sendTransform(t_sb);

        if (it->first != "Kerbin") {
            continue;
        }

        auto pos_body = m_vessel->position(body_rf);
        auto rot_body = m_vessel->rotation(body_rf);

        auto t_bv = get_transform(*m_space_center, pos_body, rot_body, body_rf, vessel_rf);
        t_bv.header.stamp = now();
        t_bv.header.frame_id = body_name_to_lower;
        t_bv.child_frame_id = "vessel";
        m_tf_broadcaster->sendTransform(t_bv);
    }

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
}