#pragma once

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <krpc/services/space_center.hpp>
#include <string>

template <typename T>
T clamp(T val, T min, T max)
{
    if (val < min) {
        return min;
    } else if (val > max) {
        return max;
    } else {
        return val;
    }
}

const char* base_name(const char*);

std::string str_lowercase(const std::string&);

geometry_msgs::msg::Vector3 tuple2vector3(const std::tuple<double, double, double>&);

geometry_msgs::msg::Quaternion tuple2quaternion(const std::tuple<double, double, double, double>&);

geometry_msgs::msg::TransformStamped get_transform(krpc::services::SpaceCenter&, std::tuple<double, double, double>, std::tuple<double, double, double, double>, krpc::services::SpaceCenter::ReferenceFrame, krpc::services::SpaceCenter::ReferenceFrame);
