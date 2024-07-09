#ifndef ASTAR_H
#define ASTAR_H
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.h"

#include <iostream>
#include <vector>
#include <queue>

#define infinity 1.0e10

struct Node{
  float cost;
  int index;
};

namespace astar_planner {
    class AstarPlanner : public nav2_core::GlobalPlanner
    {
    public:
        AstarPlanner();
        AstarPlanner(std::string name, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

        void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                        std::string name, 
                        std::shared_ptr<tf2_ros::Buffer> tf,
                        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)  
                        override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;
        nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                        const geometry_msgs::msg::PoseStamped &goal) override;
         ~AstarPlanner() override =default;
    private:
        int width_;
        int height_;
        int map_size_;
        std::vector<bool> OccuGridMap_;
        double getHeuristic(int cell_index,int goal_index);
        std::vector<int> getNeighbors(int current_cell);
        double getMoveCost(int first_index,int second_index);
        bool isInBounds(int x,int y);
        void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> &path);
        
        //publish for visualization path
        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
        std::string frame_id_;
        bool initialized_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        nav2_costmap_2d::Costmap2D *costmap_;
    };
}


#endif //ASTAR_H     //ASTAR_HPP