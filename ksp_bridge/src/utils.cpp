#include <cctype>
#include <ksp_bridge/utils.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const char* base_name(const char* file)
{
    const char* ret = file;

    while (*file++) {
        if (*file == '/') {
            ret = file + 1;
        }
    }

    return ret;
}

std::string str_lowercase(const std::string& s)
{
    auto lower = s;
    std::transform(lower.begin(), lower.end(), lower.begin(),
        [](char c) {
            return std::tolower(c);
        });

    return lower;
}

geometry_msgs::msg::Vector3 tuple2vector3(const std::tuple<double, double, double>& t)
{
    geometry_msgs::msg::Vector3 v;

    v.x = std::get<0>(t);
    v.y = std::get<1>(t);
    v.z = std::get<2>(t);

    return v;
}

geometry_msgs::msg::Quaternion tuple2quaternion(const std::tuple<double, double, double, double>& t)
{
    geometry_msgs::msg::Quaternion q;

    q.x = std::get<0>(t);
    q.y = std::get<1>(t);
    q.z = std::get<2>(t);
    q.w = std::get<3>(t);

    return q;
}

// TODO: verify this
geometry_msgs::msg::TransformStamped get_transform(krpc::services::SpaceCenter& ss, std::tuple<double, double, double> position, std::tuple<double, double, double, double> rotation, krpc::services::SpaceCenter::ReferenceFrame from, krpc::services::SpaceCenter::ReferenceFrame to)
{
    geometry_msgs::msg::TransformStamped t;

    auto new_position = ss.transform_position(position, from, to);
    auto new_rotation = ss.transform_rotation(rotation, from, to);

    auto pos1 = tuple2vector3(position);
    auto pos2 = tuple2vector3(new_position);

    auto rot1 = tuple2quaternion(rotation);
    auto rot2 = tuple2quaternion(new_rotation);

    t.transform.translation.x = pos2.x - pos1.x;
    t.transform.translation.y = pos2.y - pos1.y;
    t.transform.translation.z = pos2.z - pos1.z;

    tf2::Quaternion q1, q2, q1_inv;
    tf2::convert(rot1, q1);
    tf2::convert(rot2, q2);
    q1_inv = q1.inverse();

    auto rot = q2 * q1_inv;

    t.transform.rotation.x = rot.x();
    t.transform.rotation.y = rot.y();
    t.transform.rotation.z = rot.z();
    t.transform.rotation.w = rot.w();

    return t;
}
