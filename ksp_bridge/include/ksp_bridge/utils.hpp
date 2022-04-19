#pragma once

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

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

geometry_msgs::msg::Vector3 tuple2vector3(const std::tuple<double, double, double>& t);

geometry_msgs::msg::Quaternion tuple2quaternion(const std::tuple<double, double, double, double>& t);