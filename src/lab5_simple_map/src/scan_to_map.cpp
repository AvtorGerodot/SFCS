#include "simple_map/scan_to_map.h"
#include <rclcpp/rclcpp.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include "tf2/transform_datatypes.h"


void scan_to_map(
    const sensor_msgs::msg::LaserScan& scan, nav_msgs::msg::OccupancyGrid& map, tf2::Stamped<tf2::Transform>& tf_transform){


        // // 4. Вычисляем индексы карты
        // int x = static_cast<int>(obstacle_global.x() / map.info.resolution);
        // int y = static_cast<int>(obstacle_global.y() / map.info.resolution);

        // // 5. Записываем препятствие в карту
        // if (x >= 0 && x < map.info.width && y >= 0 && y < map.info.height)
        // {
        //     map.data[y * map.info.width + x] = 100;
        // }
        // else{
        //     map.data[y * map.info.width + x] = 0;
        // }
    }
}
