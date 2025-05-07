#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>
#include <iostream>

using namespace std::chrono_literals;

class SimpleMap : public rclcpp::Node
{
public:
    SimpleMap()
    : Node("simple_map")
    {
        // Subscriber для данных лидара
        auto laser_cb = std::bind(&SimpleMap::laserCallback, this, std::placeholders::_1);
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, laser_cb);
        
        // Publisher для карты
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/simple_map", 10);
        
        // TF Buffer и слушатель
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.frame_id = "map"; // Frame Id для карты
        map_msg.header.stamp = scan->header.stamp;
        map_msg.info.resolution = 0.1;   // Разрешение карты
        map_msg.info.width = 100;        // Ширина карты
        map_msg.info.height = 100;       // Высота карты
        map_msg.info.origin.position.x = -50.0; // Центр по X
        map_msg.info.origin.position.y = -50.0; // Центр по Y
        map_msg.info.origin.orientation.w = 1.0; // Ориентация
        map_msg.data.assign(map_msg.info.width * map_msg.info.height, -1); // Изначально карта заполнена неизвестными областями

        try
        {
            geometry_msgs::msg::TransformStamped trans = buffer_->lookupTransform(
                "map", scan->header.frame_id, scan->header.stamp, tf2::durationFromSec(0.1));
            
            double cx = trans.transform.translation.x;
            double cy = trans.transform.translation.y;
            
            int center_x = static_cast<int>((cx - map_msg.info.origin.position.x) / map_msg.info.resolution);
            int center_y = static_cast<int>((cy - map_msg.info.origin.position.y) / map_msg.info.resolution);
            
            if (center_x >= 0 && center_x < map_msg.info.width &&
                center_y >= 0 && center_y < map_msg.info.height)
            {
                size_t idx = center_y * map_msg.info.width + center_x;
                map_msg.data[idx] = 0; // Центр - свободная зона
            }
            
            for (size_t i = 0; i < scan->ranges.size(); ++i)
            {
                float range = scan->ranges[i];
                
                if (range < scan->range_min || range > scan->range_max)
                    continue;
                    
                double angle = scan->angle_min + i * scan->angle_increment;
                double x = range * cos(angle);
                double y = range * sin(angle);
                
                int idx_x = static_cast<int>((x - map_msg.info.origin.position.x) / map_msg.info.resolution);
                int idx_y = static_cast<int>((y - map_msg.info.origin.position.y) / map_msg.info.resolution);
                
                if (idx_x >= 0 && idx_x < map_msg.info.width &&
                    idx_y >= 0 && idx_y < map_msg.info.height)
                {
                    size_t idx = idx_y * map_msg.info.width + idx_x;
                    map_msg.data[idx] = 100; // Место препятствия
                }
            }
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_ERROR(get_logger(), "Transformation error: %s", ex.what());
            return;
        }
        
        map_pub_->publish(map_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleMap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
