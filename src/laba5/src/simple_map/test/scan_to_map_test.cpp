#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "simple_map/scan_to_map.h"
#include "geometry_msgs/msg/transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

TEST(TestExample, MathOperations) {
  EXPECT_EQ(pow(2, 2), 4);
  EXPECT_TRUE(cos(M_PI/45) < 1.0);
  EXPECT_FLOAT_EQ(sin(M_PI/6), 0.5);
}

TEST(ScanToMap, SimpleMap) {
    sensor_msgs::msg::LaserScan scan;
    // Устанавливаем только одну точку в нуле
    scan.ranges.push_back(0.05);
    scan.angle_min = 0;
    scan.angle_max = 0;
    scan.angle_increment = 0;

    nav_msgs::msg::OccupancyGrid map;

    map.info.height = 100;
    map.info.width = 100;
    map.info.resolution = 0.1;
    map.info.origin.position.x = -(map.info.width * map.info.resolution) / 2;
    map.info.origin.position.y = -(map.info.height * map.info.resolution) / 2;

    // Изменяем размер вектора данных карты и заполняем его значением -1 (неизвестно)
    map.data.resize(map.info.height * map.info.width, -1);

    // Создаем identity-трансформ
    geometry_msgs::msg::Transform transform;
    transform.rotation.w = 1.0; // identity rotation

    scan_to_map(scan, map, transform);
    
    // Проверяем центральную точку
    EXPECT_EQ(map.data[map.info.width / 2 + (map.info.height / 2 - 1) * map.info.width], 100);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
