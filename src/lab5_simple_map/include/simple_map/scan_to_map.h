#pragma once

#include <rclcpp/rclcpp.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include "tf2/transform_datatypes.h"

/**
 * @brief Преобразует данные лазерного скана в карту занятости
 * 
 * @param scan Входные данные лазерного скана (ROS2 LaserScan сообщение)
 * @param map Выходная карта занятости (ROS2 OccupancyGrid сообщение)
 * @param transform Трансформация из системы координат лазера в систему координат карты
 */
void scan_to_map(
    const sensor_msgs::msg::LaserScan& scan, nav_msgs::msg::OccupancyGrid& map, tf2::Stamped<tf2::Transform>& tf_transform);
