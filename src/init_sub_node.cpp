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

float init_pose_point[3]={0};

// 地图起点数据订阅者
class navInitSubscriber : public rclcpp::Node{
public:

  navInitSubscriber(std::string name);

  // void get_initposepoint(float init[]){
  //   init[0] = init_pose_point[0];
  //   init[1] = init_pose_point[1];
  //   init[2] = init_pose_point[2];
  // }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber;

  // 回调函数
  void data_callback(const geometry_msgs::msg::PoseWithCovarianceStamped msg);
};

// 地图起点数据订阅者构造函数
navInitSubscriber::navInitSubscriber(std::string name) : Node(name){
  RCLCPP_INFO(this->get_logger(), "%s节点已经启动", name.c_str());
  subscriber = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10000, 
  std::bind(&navInitSubscriber::data_callback, this, placeholders::_1));
  
}
// 地图起点数据订阅者回调函数
void navInitSubscriber::data_callback(const geometry_msgs::msg::PoseWithCovarianceStamped msg){
  init_pose_point[0] = 1;
  init_pose_point[1] = msg.pose.pose.position.x;
  init_pose_point[2] = msg.pose.pose.position.y;
  RCLCPP_INFO(this->get_logger(), "起点数据已经调用");
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = make_shared<navInitSubscriber>("goal_sub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
