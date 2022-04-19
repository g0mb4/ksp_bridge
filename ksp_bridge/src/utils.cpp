#include "ksp_bridge/utils.hpp"

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
