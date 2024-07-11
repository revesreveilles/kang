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

float goal_pose_point[3]={0};

using namespace std;

// 地图终点数据订阅函数
class navGoalSubscriber : public rclcpp::Node{
public:
  navGoalSubscriber(std::string name);

  // void get_goalposepoint(float goal[]){
  //   goal[0] = goal_pose_point[0];
  //   goal[1] = goal_pose_point[1];
  //   goal[2] = goal_pose_point[2];
  // }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber;

  // 回调函数
  void data_callback(const geometry_msgs::msg::PoseStamped msg);
};

// 地图终点数据订阅者构造函数
navGoalSubscriber::navGoalSubscriber(std::string name) : Node(name){
  RCLCPP_INFO(this->get_logger(), "%s节点已经启动", name.c_str());
  subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10000, 
  std::bind(&navGoalSubscriber::data_callback, this, placeholders::_1));
}

// 地图终点数据订阅者回调函数
void navGoalSubscriber::data_callback(const geometry_msgs::msg::PoseStamped msg){
  goal_pose_point[0] = 1;
  goal_pose_point[1] = msg.pose.position.x;
  goal_pose_point[2] = msg.pose.position.y;
  RCLCPP_INFO(this->get_logger(), "终点数据已经调用");
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = make_shared<navGoalSubscriber>("goal_sub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}