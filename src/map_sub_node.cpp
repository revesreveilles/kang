#include <math.h>
#include <vector>
#include <list>
#include <clocale>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std;

class navSubscriber : public rclcpp::Node{
public:
  int origin_x = 0;
  int origin_y = 0;
  float resolution = 0;
  int width = 0;
  int height = 0;
  vector<vector<int>> map_data;
  navSubscriber();
  navSubscriber(std::string name);
private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber;
  // 回调函数
  void data_callback(const nav_msgs::msg::OccupancyGrid msg);
};

// 地图数据订阅节点的构造函数
navSubscriber::navSubscriber():Node(""){}

navSubscriber::navSubscriber(std::string name):Node(name){
  RCLCPP_INFO(this->get_logger(), "%s节点已经启动", name.c_str());
  subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10000, 
  bind(&navSubscriber::data_callback, this, placeholders::_1));
}

// 地图数据订阅节点的回调函数
void navSubscriber::data_callback(const nav_msgs::msg::OccupancyGrid msg){
    RCLCPP_INFO(this->get_logger(), "地图订阅获取到数据");
    origin_x = msg.info.origin.position.x;
    origin_y = msg.info.origin.position.y;
    resolution = msg.info.resolution;
    width = msg.info.width;
    height = msg.info.height;

    for(int i = 0; i < width; i++){
      map_data.push_back(vector<int>());
    }
    for(int i = 0; i < width; i++){
      for(int j = 0; j < width; j++){
        map_data[i].push_back(msg.data[j*width+i]);
      }
    }
    
  }

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = make_shared<navSubscriber>("goal_sub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
