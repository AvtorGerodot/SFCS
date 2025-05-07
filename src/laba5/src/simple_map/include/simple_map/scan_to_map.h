#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform.hpp>

/**
 * @brief Преобразует данные лазерного скана в карту занятости
 * 
 * @param scan Входные данные лазерного скана (ROS2 LaserScan сообщение)
 * @param map Выходная карта занятости (ROS2 OccupancyGrid сообщение)
 * @param transform Трансформация из системы координат лазера в систему координат карты
 */
void scan_to_map(
    const sensor_msgs::msg::LaserScan& scan,
    nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Transform& transform);
