#include "astar_planner.h"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>


//plugin register
PLUGINLIB_EXPORT_CLASS(astar_planner::AstarPlanner, nav2_core::GlobalPlanner)

namespace astar_planner{
  AstarPlanner::AstarPlanner(): initialized_(false) {}
  AstarPlanner::AstarPlanner(std::string name, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    : initialized_(false)
    {
      configure(rclcpp_lifecycle::LifecycleNode::WeakPtr(),name,nullptr,costmap_ros);
    }
    
  void AstarPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                    std::string name,
                    std::shared_ptr<tf2_ros::Buffer> tf,
                    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
      (void) name;
      (void) tf;
      if(!initialized_)
      {
        auto node = parent.lock();
        if(!node)
        {
          throw std::runtime_error("Unable to lock node");
        }
        costmap_ros_ =costmap_ros;
        costmap_ =costmap_ros->getCostmap();
        width_ =costmap_->getSizeInCellsX();
        height_ =costmap_->getSizeInCellsY();
        map_size_ =width_*height_;
        OccuGridMap_.resize(map_size_);

        for(int i =0;i<width_;i++)
        {
          for(int j =0;j<height_;j++)
          {
            unsigned int cost =static_cast<int>(costmap_->getCost(j,i));
            OccuGridMap_[i*width_+j]=(cost==0);
          }
        }
        frame_id_=costmap_ros_->getGlobalFrameID();
        plan_pub_ =node->create_publisher<nav_msgs::msg::Path>("planned_path",10);
        initialized_ =true;
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"THis planner has already been initialized");
      }
    }
    void AstarPlanner::cleanup()
    {
      plan_pub_.reset();
      initialized_ =false;
    }
    void AstarPlanner::activate()
    {
      plan_pub_->on_activate();
    }
    void AstarPlanner::deactivate()
    {
      plan_pub_->on_deactivate();
    }

    nav_msgs::msg::Path AstarPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                                                const geometry_msgs::msg::PoseStamped &goal)
      {
        std::vector<geometry_msgs::msg::PoseStamped> plan;
        if(!initialized_)
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"AstarPlanner has not been initialized!");
          return nav_msgs::msg::Path();
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Got a start:(%.2f,%.2f),and a goal:(%.2f,%.2f)",
                  start.pose.position.x,start.pose.position.y,goal.pose.position.x,goal.pose.position.y);
        double wx =start.pose.position.x;
        double wy =start.pose.position.y;
        unsigned int start_x,start_y;
        costmap_->worldToMap(wx,wy,start_x,start_y);
        int start_index =costmap_->getIndex(start_x,start_y);
        wx =goal.pose.position.x;
        wy =goal.pose.position.y;
        unsigned int goal_x,goal_y;
        costmap_->worldToMap(wx,wy,goal_x,goal_y);
        int goal_index =costmap_->getIndex(goal_x,goal_y);

        std::vector<float> gCosts(map_size_,infinity);
        std::vector<int> cameFrom(map_size_,-1);

        std::multiset<Node> priority_costs;
        gCosts[start_index] =0;
        Node currentNode;
        currentNode.cost=gCosts[start_index]+0;
        priority_costs.insert(currentNode);

        plan.clear();

        while(!priority_costs.empty())
        {
          currentNode =*priority_costs.begin();
          priority_costs.erase(priority_costs.begin());
          if(currentNode.index ==goal_index)
          {
            break;
          }
          //Get neighbors
          std::vector<int> neighborIndexes =getNeighbors(currentNode.index);
          for(size_t i=0;i<neighborIndexes.size();i++)
          {
            if(cameFrom[neighborIndexes[i]]==-1)
            {
              gCosts[neighborIndexes[i]]=gCosts[currentNode.index]+getMoveCost(currentNode.index,neighborIndexes[i]);
              Node nextNode;
              nextNode.index =neighborIndexes[i];
              //nextNode.cost=gCosts[neighborIndexes[i]];  //Dijkstra Algorithm
              nextNode.cost =gCosts[neighborIndexes[i]]+getHeuristic(neighborIndexes[i],goal_index);
              cameFrom[neighborIndexes[i]]= currentNode.index;
              priority_costs.insert(nextNode);
            }
          }
        }
        if(cameFrom[goal_index]==-1)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Goal not reachable,failed to plan!");
          return nav_msgs::msg::Path();
        }
        if(start_index ==goal_index)
          return nav_msgs::msg::Path();

        std::vector<int> bestPath;
        int currentNode_index =goal_index;
        while(currentNode_index !=start_index)
        {
          bestPath.push_back(cameFrom[currentNode_index]);
          currentNode_index =cameFrom[currentNode_index];
        }
        std::reverse(bestPath.begin(),bestPath.end());

        rclcpp::Time plan_time =rclcpp::Clock().now();
        for(int index:bestPath)
        {
          unsigned int tmp1,tmp2;
          costmap_->indexToCells(index,tmp1,tmp2);
          double x,y;
          costmap_->mapToWorld(tmp1,tmp2,x,y);

          geometry_msgs::msg::PoseStamped pose;
          pose.header.stamp =plan_time;
          pose.header.frame_id =costmap_ros_->getGlobalFrameID();
          pose.pose.position.x =x;
          pose.pose.position.x =x;
          pose.pose.position.z =0.0;
          pose.pose.orientation.x =0.0;
          pose.pose.orientation.y =0.0;
          pose.pose.orientation.x =0.0;
          pose.pose.orientation.w =1.0;

          plan.push_back(pose);
        }
        plan.push_back(goal);
        publishPlan(plan);
        return nav_msgs::msg::Path();
      }
      double AstarPlanner::getMoveCost(int firstIndex,int secondIndex)
      {
        unsigned int tmp1,tmp2;
        costmap_->indexToCells(firstIndex,tmp1,tmp2);
        int firstXCord =tmp1,firstYCord =tmp2;
        costmap_->indexToCells(secondIndex,tmp1,tmp2);
        int secondXCord =tmp1,secondYCord=tmp2;

        int difference =std::abs(firstXCord- secondXCord)+std::abs(firstYCord-secondYCord);
        if(difference !=1&&difference!=2)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Astar Planner:Error in getMoveCost --difference not valid");
          return 1.0;
        }
        return (difference==1)?1.0:1.4;
      }

      double AstarPlanner::getHeuristic(int cell_index,int goal_index)
      {
        unsigned int tmp1,tmp2;
        costmap_->indexToCells(cell_index,tmp1,tmp2);
        int startX=tmp1,startY =tmp2;
        costmap_->indexToCells(goal_index,tmp1,tmp2);
        int goalX= tmp1,goalY= tmp2;
        return std::abs(goalY-startY)+std::abs(goalX -startX);
      }
      
      bool AstarPlanner::isInBounds(int x,int y)
      {
        return (x>=0&&y>=0&&x<height_&&y<width_);
      }

      std::vector<int> AstarPlanner::getNeighbors(int currentNode_cell)
      {
        std::vector<int> neighborIndexes;
        for(int i=-1;i<=1;i++)
        {
          for(int j=-1;j<=1;j++)
          {
            unsigned tmp1,tmp2;
            costmap_->indexToCells(currentNode_cell,tmp1,tmp2);
            int nextX =tmp1+i;
            int nextY =tmp2+j;
            int nextIndex =costmap_->getIndex(nextX,nextY);
            if(!(i==0&&j==0)&& isInBounds(nextX,nextY)&&OccuGridMap_[nextIndex])
            {
                neighborIndexes.push_back(nextIndex);
            }
          }
        }
        return neighborIndexes;
      }
      
      void AstarPlanner::publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path) 
      {
        if (!initialized_) 
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
          return;
        }
        //create msgs for planned path
        nav_msgs::msg::Path path_msg;
        path_msg.poses.resize(path.size());

        path_msg.header.frame_id = frame_id_;
        path_msg.header.stamp = rclcpp::Clock().now();

        //Extract the plan in world co-cordinates, assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) 
        {
          path_msg.poses[i] = path[i];
        }

        plan_pub_->publish(path_msg);
      }
};
bool operator <(const Node& x,const Node& y)
{
  return x.cost<y.cost;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto planner_node= std::make_shared<rclcpp::Node>("astar_planner_node");
    auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");

    auto astar_planner = std::make_shared<astar_planner::AstarPlanner>();
    
    rclcpp::spin(planner_node);

    rclcpp::shutdown();
    return 0;
}
