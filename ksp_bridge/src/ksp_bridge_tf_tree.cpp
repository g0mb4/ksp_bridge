#include "ksp_bridge/ksp_bridge.hpp"
#include "ksp_bridge/utils.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

void KSPBridge::send_tf_tree()
{
    // world -> sun, world = sun
    geometry_msgs::msg::TransformStamped t_ws;
    t_ws.header.stamp = now();
    t_ws.header.frame_id = "world";
    t_ws.child_frame_id = "sun";

    m_tf_broadcaster->sendTransform(t_ws);

    // sun -> kerbin
    auto sun_rf = m_celestial_bodies["Sun"].reference_frame();
    auto kerbin_rf = m_celestial_bodies["Kerbin"].reference_frame();

    auto pos_sun = m_vessel->position(sun_rf);
    auto rot_sun = m_vessel->rotation(sun_rf);

    auto t_sk = get_transform(*m_space_center, pos_sun, rot_sun, sun_rf, kerbin_rf);
    t_sk.header.stamp = now();
    t_sk.header.frame_id = "sun";
    t_sk.child_frame_id = "kerbin";
    m_tf_broadcaster->sendTransform(t_sk);

    // kerbin -> vessel
    auto vessel_rf = m_vessel->reference_frame();

    auto pos_kerbin = m_vessel->position(kerbin_rf);
    auto rot_kerbin = m_vessel->rotation(kerbin_rf);

    auto t_kv = get_transform(*m_space_center, pos_kerbin, rot_kerbin, kerbin_rf, vessel_rf);
    t_kv.header.stamp = now();
    t_kv.header.frame_id = "kerbin";
    t_kv.child_frame_id = "vessel";
    m_tf_broadcaster->sendTransform(t_kv);

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