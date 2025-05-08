#include "simple_map/scan_to_map.h"
#include <rclcpp/rclcpp.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


void scan_to_map(
    const sensor_msgs::msg::LaserScan& scan, 
    nav_msgs::msg::OccupancyGrid& map, 
    const geometry_msgs::msg::Transform& transform)
{
    // Преобразование Transform в tf2::Transform
    tf2::Transform tf_transform;
    tf2::fromMsg(transform, tf_transform);

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
        // Вычисление угла для текущего измерения
        double angle = scan.angle_min + scan.angle_increment * i;
        
        // Преобразование точки из СК лазера в СК карты
        tf2::Vector3 point_in_laser_frame(
            scan.ranges[i] * std::cos(angle),
            scan.ranges[i] * std::sin(angle),
            0.0);
        
        tf2::Vector3 point_in_map_frame = tf_transform * point_in_laser_frame;
        
        // Перевод в координаты относительно начала карты
        point_in_map_frame -= tf2::Vector3(
            map.info.origin.position.x,
            map.info.origin.position.y,
            0.0);

        // Проверка, что точка попадает в границы карты
        if (point_in_map_frame.x() >= 0 && point_in_map_frame.y() >= 0 &&
            point_in_map_frame.x() < map.info.resolution * map.info.width && 
            point_in_map_frame.y() < map.info.resolution * map.info.height) 
        {
            // Вычисление индексов ячейки карты
            std::size_t x = static_cast<std::size_t>(point_in_map_frame.x() / map.info.resolution);
            std::size_t y = static_cast<std::size_t>(point_in_map_frame.y() / map.info.resolution);
            
            // Здесь можно добавить логику обновления карты
            // Например: map.data[y * map.info.width + x] = 100; // для занятых ячеек
        }
    }
}
