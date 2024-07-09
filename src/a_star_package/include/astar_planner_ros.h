#ifndef ASTAR_GLOBAL_PLANNER_ROS_
#define ASTAR_GLOBAL_PLANNER_ROS_

#include <rclcpp/rclcpp.hpp>
#include "astar_planner.h"

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <nav2_core/global_planner.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <angles/angles.h>
// #include <base_local_planner/world_model.h>
// #include <base_local_planner/costmap_model.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using std::string;

namespace astar_global_planner
{
    /*Astar 算法与外接接口*/
    class AstarGlobalPlannerRos : public nav2_core::GlobalPlanner {
        public:
        /*Default constructor for the Astar Planner Ros object*/
        AstarGlobalPlannerRos();
        /*constructor for Astar Planner Ros object*/
        AstarGlobalPlannerRos(std::string name, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
        /*constructor for Astar Planner Ros object*/
        AstarGlobalPlannerRos(std::string name, nav2_costmap_2d::Costmap2D* costmap, std::string global_frame);

        /** overridden classes from interface nav2_core::GlobalPlanner **/
        /*Initialization function for the A star Planner object*/
        void configure(
          const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
          std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
          std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        void cleanup() override;
        void activate() override;
        void deactivate() override;

        /*Given a goal pose in the world, compute a plan*/
        nav_msgs::msg::Path createPlan(
          const geometry_msgs::msg::PoseStamped & start,
          const geometry_msgs::msg::PoseStamped & goal) override;

        void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path);
        bool makePlanService(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
                             std::shared_ptr<nav_msgs::srv::GetPlan::Response> resp);        
        
        private:
        /*函数*/
        void clearRobotCell(const geometry_msgs::msg::PoseStamped& global_pose, unsigned int mx, unsigned int my);
        void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
        bool worldToMap(double wx, double wy, int& mx, int& my);
        void mapToWorld(double mx, double my, double& wx, double& wy);
        

        /*变量*/
        nav2_costmap_2d::Costmap2D* costmap_;

        // 创建一个路径规划的指针，用于调用路径规划的函数与变量
        std::shared_ptr<astar_planner::AstarPlanner> planner_;
        
        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
        rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr make_plan_srv_;

        std::string frame_id_;

        std::mutex mutex_;
        //需要从参数服务器上加载的，在launch文件中配置相应的yaml文件上传到参数服务器
        bool initialized_;  // 初始化的标志
        bool visualize_potential_; // 是否通过PointCloud2消息在rviz中可视化潜在的区域
        bool allow_unknown_; // 是否允许NavFn规划的路径通过未知区域，默认是true
        bool outline_map_;
        double planner_window_x_, planner_window_y_; // 规划窗口的大小，默认是0。(按:从源码中看，好像没用到)
        double default_tolerance_; //到达终点的容忍度。当规划的路径点到目标点的距离小于该参数描述的值的时候，就认为路径点基本到达了目标点
        double conver_offset;
    };
};

#endif // ASTAR_GLOBAL_PLANNER_ROS_
