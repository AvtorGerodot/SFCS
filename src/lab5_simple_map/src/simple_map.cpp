#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "simple_map/scan_to_map.h"

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

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {

        geometry_msgs::msg::TransformStamped tfGeom;
        const std::string &laser_frame = scan->header.frame_id;
        const rclcpp::Time &laser_stamp = scan->header.stamp;
        try
        {
            //if (!tfListener->waitForTransform(map_frame,
            //                                   laser_frame,
            //                                   stamp,
            //                                   rclcpp::Duration(0.1)))
            // {
            //     RCLCPP_ERROR(this->get_logger(),"no transform to scan %s", laser_frame);
            //     return false;
            // }
            tfGeom = buffer_->lookupTransform(map_frame, laser_frame, laser_stamp, rclcpp::Duration(0.1));
        }
        catch (tf2::TransformException &e)
        {
            RCLCPP_ERROR(this->get_logger(),"got tf exception %s", e.what());
            return;
        }
        tf2::Stamped<tf2::Transform> tf_transform;
        tf2::convert(tfGeom, tf_transform);  //# note that tf2 is missing child_frame_id

        // создаем сообщение карты
        nav_msgs::msg::OccupancyGrid map_msg;
        // заполняем информацию о карте - готовим сообщение
        prepareMapMessage(map_msg, laser_stamp);
        

        // положение центра дальномера в СК дальномера
        tf2::Vector3 zero_pose(0, 0, 0);
        // положение дальномера в СК карты=
        tf2::Vector3 scan_pose = tf_transform * zero_pose;
        //tf2::Vector3 scan_pose = tfGeom(zero_pose);
        RCLCPP_INFO(this->get_logger(),"scan pose %d, %d", scan_pose.x(), scan_pose.y());

        // задаем начало карты так, чтобы сканнер находился в центре карты
        map_msg.info.origin.position.x = scan_pose.x() - map_width * map_resolution / 2.0;
        map_msg.info.origin.position.y = scan_pose.y() - map_height * map_resolution / 2.0;

        // индексы карты, соответствующие положению центра лазера
        int y = (scan_pose.y() - map_msg.info.origin.position.y) / map_resolution;
        int x = (scan_pose.x() - map_msg.info.origin.position.x) / map_resolution;
        RCLCPP_INFO(this->get_logger(),"publish map  %d, %d", x, y);
        // в клетку карты соотвтествующую центру лазера - записываем значение 100
        map_msg.data[y * map_width + x] = 100;

        int map_idx_max = map_width * map_height;
        for (std::size_t i = 0; i < scan->ranges.size(); ++i)
        {
            // 1. Пропускаем невалидные значения
            if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max)
                continue;
    
            // 2. Вычисляем координаты препятствия
            double angle = scan->angle_min + scan->angle_increment * i;
            tf2::Vector3 obstacle_local(
                scan->ranges[i] * cos(angle),
                scan->ranges[i] * sin(angle),
                0);
    
            // 3. Преобразуем в глобальные координаты
            tf2::Vector3 obstacle_global = tf_transform * obstacle_local;
            obstacle_global -= tf2::Vector3(
                map_msg.info.origin.position.x,
                map_msg.info.origin.position.y,
                0);
            // вычисляем позицию препятствия в системе координат ЛД
            tf2::Vector3 obstacle_pose(scan->ranges[i] * cos(angle), scan->ranges[i] * sin(angle), 0.0);
            // Шаг для прохода по лучу
            double step = 0.1;
            for (double r = scan->range_min; r < scan->ranges[i] - step; r += step)
            {
                // Точка в ДСК ЛД
                tf2::Vector3 free_pos(r * cos(angle), r * sin(angle), 0.0);
                // Точка в ДСК Карты
                tf2::Vector3 free_pos_map = tf_transform * free_pos;
                // коорд точки карты
                int free_x = (free_pos_map.x() - map_msg.info.origin.position.x) / map_resolution;
                int free_y = (free_pos_map.y() - map_msg.info.origin.position.y) / map_resolution;
                // индекс в массиве карты
                int map_free_idx = free_y * map_width + free_x;
                // проверяем, что ячейка не находится за пределами карты
                if (map_free_idx > 0 && map_free_idx < map_idx_max)
                {
                    map_msg.data[map_free_idx] = 0;
                }
            }
            // вычисляем позицию препятствия в системе координат карты
            tf2::Vector3 obstacle_pose_map = tf_transform * obstacle_pose;
            // индексы ячейки, соответствующей позиции препятствия
            int obstacle_x = (obstacle_pose_map.x() -
                              map_msg.info.origin.position.x) /
                             map_resolution;
            int obstacle_y = (obstacle_pose_map.y() -
                              map_msg.info.origin.position.y) /
                             map_resolution;
            int map_idx = obstacle_y * map_width + obstacle_x;
            // проверяем, что ячейка не находится за пределами карты
            if (map_idx > 0 && map_idx < map_idx_max)
            {
                map_msg.data[map_idx] = 100;
            }
        }

        // публикуем сообщение с построенной картой
        map_pub_->publish(map_msg);
    }

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
