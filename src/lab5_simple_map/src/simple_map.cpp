#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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
        // Инициализация подписчика на данные лидара
        laserSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/base_scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                this->laserCallback(msg);
            });
        
        // Инициализация публикатора карты
        mapPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("simple_map", 10);
        
        // Инициализация системы преобразований координат (TF2)
        buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    }

private:
    // Подготовка сообщения с картой
    void prepareMapMessage(nav_msgs::msg::OccupancyGrid& map_msg, const rclcpp::Time& stamp)
    {
        // Установка параметров карты
        map_msg.header.frame_id = map_frame;   // Система координат карты
        map_msg.header.stamp = stamp;          // Временная метка
        
        // Геометрические параметры карты
        map_msg.info.height = map_height;     // Высота в клетках
        map_msg.info.width = map_width;       // Ширина в клетках
        map_msg.info.resolution = map_resolution; // Разрешение
        
        // Инициализация данных карты (-1 = неизвестная область)
        map_msg.data.resize(map_height*map_width, -1);
    }

    // Обработчик данных лидара
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        geometry_msgs::msg::TransformStamped tfGeom;
        const std::string &laser_frame = scan->header.frame_id;
        const rclcpp::Time &laser_stamp = scan->header.stamp;
        
        try {
            // Получение преобразования между системами координат
            tfGeom = buffer->lookupTransform(
                map_frame, 
                laser_frame, 
                rclcpp::Time(0), // Время преобразования - последнее доступное
                rclcpp::Duration(100ms) // Таймаут увеличен до 100 мс
            );
        }
        catch (tf2::TransformException &e) {
            RCLCPP_ERROR(this->get_logger(),"Ошибка преобразования: %s", e.what());
            return;
        }
        
        // Конвертация TransformStamped в tf2::Transform
        tf2::Stamped<tf2::Transform> tfTransform;
        tf2::convert(tfGeom, tfTransform);

        // Создание нового сообщения карты
        nav_msgs::msg::OccupancyGrid map_msg;
        prepareMapMessage(map_msg, laser_stamp);
        
        // Расчет позиции робота в СК карты
        tf2::Vector3 scan_pose = tfTransform * tf2::Vector3(0, 0, 0);
        RCLCPP_INFO(this->get_logger(),"Позиция робота: x=%.2f, y=%.2f", 
                   scan_pose.x(), scan_pose.y());

        // Центрирование карты относительно позиции робота
        map_msg.info.origin.position.x = scan_pose.x() - map_width * map_resolution / 2.0;
        map_msg.info.origin.position.y = scan_pose.y() - map_height * map_resolution / 2.0;

        // Максимальный индекс карты
        int map_idx_max = map_width * map_height;
        
        // Обработка всех измерений лидара
        for (std::size_t i = 0; i < scan->ranges.size(); ++i)
        {
            // Пропуск невалидных измерений
            if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max)
                continue;
    
            // Расчет угла для текущего луча
            double angle = scan->angle_min + scan->angle_increment * i;
            
            // Обработка свободного пространства (лучи до препятствий)
            double step = 0.1;  // Шаг дискретизации луча (10 см)
            for (double r = scan->range_min; r < scan->ranges[i] - step; r += step)
            {
                // Преобразование координат свободного пространства
                tf2::Vector3 free_pos(r * cos(angle), r * sin(angle), 0.0);
                tf2::Vector3 free_pos_map = tfTransform * free_pos;
                
                // Расчет индексов ячейки
                int free_x = (free_pos_map.x() - map_msg.info.origin.position.x) / map_resolution;
                int free_y = (free_pos_map.y() - map_msg.info.origin.position.y) / map_resolution;
                
                // Проверка границ карты и обновление данных
                if (free_x >= 0 && free_x < map_width && 
                    free_y >= 0 && free_y < map_height)  {
                    map_msg.data[free_y * map_width + free_x] = 0;  // 0% - свободное пространство
                }
            }
            
            // Обработка препятствия
            tf2::Vector3 obstacle_pose(scan->ranges[i] * cos(angle), scan->ranges[i] * sin(angle), 0.0);
            tf2::Vector3 obstacle_pose_map = tfTransform * obstacle_pose;
            
            // Расчет индексов ячейки с препятствием
            int obstacle_x = (obstacle_pose_map.x() - map_msg.info.origin.position.x) / map_resolution;
            int obstacle_y = (obstacle_pose_map.y() - map_msg.info.origin.position.y) / map_resolution;
            
            // Проверка границ карты и обновление данных
            if (obstacle_x >= 0 && obstacle_x < map_width && 
                obstacle_y >= 0 && obstacle_y < map_height) {
                map_msg.data[obstacle_y * map_width + obstacle_x] = 100;  // 100% - препятствие
            }
        }

        // Расчет индексов центральной ячейки
        int y = (scan_pose.y() - map_msg.info.origin.position.y) / map_resolution;
        int x = (scan_pose.x() - map_msg.info.origin.position.x) / map_resolution;
        RCLCPP_INFO(this->get_logger(),"Центр карты: [%d, %d]", x, y);

        // Отметка позиции робота на карте (100% вероятность препятствия)
        map_msg.data[y * map_width + x] = 100;

        // Публикация обновленной карты
        mapPublisher->publish(map_msg);
    }

    // Параметры карты
    double map_resolution = 0.1;   
    int map_width = 100;
    int map_height = 100;
    std::string map_frame = "odom";

    // ROS2 интерфейсы
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSubscriber;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher;
    std::shared_ptr<tf2_ros::Buffer> buffer;
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