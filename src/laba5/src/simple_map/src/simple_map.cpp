#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


using namespace std::chrono_literals;

class SimpleMap : public rclcpp::Node
{
public:
    SimpleMap(): Node("simple_map")
    {
        // Subscriber для данных лидара
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/base_scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                this->laserCallback(msg);
            });

        
        // Publisher для карты
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("simple_map", 10);
        
        // TF Buffer и слушательs
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    }

private:
    void prepareMapMessage(nav_msgs::msg::OccupancyGrid& map_msg, const rclcpp::Time& stamp)
    {
        map_msg.header.frame_id = map_frame;
        map_msg.header.stamp = stamp;
    
        map_msg.info.height = map_height;
        map_msg.info.width = map_width;
        map_msg.info.resolution = map_resolution;
    
        // изменяем размер вектора, который является хранилищем данных карты, и заполняем его значением (-1) - неизвестное значение
        map_msg.data.resize(map_height*map_width, -1);
    }

    bool determineScanTransform(tf2::Stamped<tf2::Transform>& scanTransform,
                                const rclcpp::Time &stamp,
                                const std::string &laser_frame)
    {
        try
        {
            // if (!tfListener->waitForTransform(map_frame,
            //                                   laser_frame,
            //                                   stamp,
            //                                   rclcpp::Duration(0.1)))
            // {
            //     RCLCPP_ERROR(this->get_logger(),"no transform to scan %s", laser_frame);
            //     return false;
            // }
            buffer_->lookupTransform(map_frame, laser_frame, stamp);
        }
        catch (tf2::TransformException &e)
        {
            RCLCPP_ERROR(this->get_logger(),"got tf exception %s", e.what());
            return false;
        }
        return true;
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        tf2::Stamped<tf2::Transform> scanTransform;
        const std::string &laser_frame = scan->header.frame_id;
        const rclcpp::Time &laser_stamp = scan->header.stamp;
        if (!determineScanTransform(scanTransform, laser_stamp, laser_frame))
        {
            return;
        }

        // создаем сообщение карты
        nav_msgs::msg::OccupancyGrid map_msg;
        // заполняем информацию о карте - готовим сообщение
        prepareMapMessage(map_msg, laser_stamp);

        // положение центра дальномера в СК дальномера
        tf2::Vector3 zero_pose(0, 0, 0);
        // положение дальномера в СК карты
        tf2::Vector3 scan_pose = scanTransform(zero_pose);
        RCLCPP_INFO(this->get_logger(),"scan pose %d, %d", scan_pose.x(), scan_pose.y());

        // задаем начало карты так, чтобы сканнер находился в центре карты
        map_msg.info.origin.position.x = scan_pose.x() - map_width * map_resolution / 2.0;
        map_msg.info.origin.position.y = scan_pose.y() - map_height * map_resolution / 2.0;

        // индексы карты, соответствующие положению центра лазера
        int y = (scan_pose.y() - map_msg.info.origin.position.y) / map_resolution;
        int x = (scan_pose.x() - map_msg.info.origin.position.x) / map_resolution;
        RCLCPP_INFO(this->get_logger(),"publish map  %d, %d", x, y);
        // в клетку карты соотвтествующую центру лазера - записываем значение 0
        map_msg.data[y * map_width + x] = 0;

        // публикуем сообщение с построенной картой
        map_pub_->publish(map_msg);
    }

    // void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    // {
    //     nav_msgs::msg::OccupancyGrid map_msg;
    //     map_msg.header.frame_id = "map"; // Frame Id для карты
    //     map_msg.header.stamp = scan->header.stamp;
    //     map_msg.info.resolution = 0.1;   // Разрешение карты
    //     map_msg.info.width = 100;        // Ширина карты
    //     map_msg.info.height = 100;       // Высота карты
    //     map_msg.info.origin.position.x = -50.0; // Центр по X
    //     map_msg.info.origin.position.y = -50.0; // Центр по Y
    //     map_msg.info.origin.orientation.w = 1.0; // Ориентация
    //     map_msg.data.assign(map_msg.info.width * map_msg.info.height, -1); // Изначально карта заполнена неизвестными областями

    //     try
    //     {
    //         geometry_msgs::msg::TransformStamped trans = buffer_->lookupTransform(
    //             "map", scan->header.frame_id, scan->header.stamp, tf2::durationFromSec(0.1));
            
    //         double cx = trans.transform.translation.x;
    //         double cy = trans.transform.translation.y;
            
    //         int center_x = static_cast<int>((cx - map_msg.info.origin.position.x) / map_msg.info.resolution);
    //         int center_y = static_cast<int>((cy - map_msg.info.origin.position.y) / map_msg.info.resolution);
            
    //         if (center_x >= 0 && center_x < map_msg.info.width &&
    //             center_y >= 0 && center_y < map_msg.info.height)
    //         {
    //             size_t idx = center_y * map_msg.info.width + center_x;
    //             map_msg.data[idx] = 0; // Центр - свободная зона
    //         }
            
    //         for (size_t i = 0; i < scan->ranges.size(); ++i)
    //         {
    //             float range = scan->ranges[i];
                
    //             if (range < scan->range_min || range > scan->range_max)
    //                 continue;
                    
    //             double angle = scan->angle_min + i * scan->angle_increment;
    //             double x = range * cos(angle);
    //             double y = range * sin(angle);
                
    //             int idx_x = static_cast<int>((x - map_msg.info.origin.position.x) / map_msg.info.resolution);
    //             int idx_y = static_cast<int>((y - map_msg.info.origin.position.y) / map_msg.info.resolution);
                
    //             if (idx_x >= 0 && idx_x < map_msg.info.width &&
    //                 idx_y >= 0 && idx_y < map_msg.info.height)
    //             {
    //                 size_t idx = idx_y * map_msg.info.width + idx_x;
    //                 map_msg.data[idx] = 100; // Место препятствия
    //             }
    //         }
    //     }
    //     catch (tf2::TransformException& ex)
    //     {
    //         RCLCPP_ERROR(get_logger(), "Transformation error: %s", ex.what());
    //         return;
    //     }
        
    //     map_pub_->publish(map_msg);
    // }

    //разрешение карты
    double map_resolution = 0.1;
    // размер карты в клетках
    int map_width = 100;
    int map_height = 100;

    //имя для СК карты
    std::string map_frame = "odom";

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleMap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
