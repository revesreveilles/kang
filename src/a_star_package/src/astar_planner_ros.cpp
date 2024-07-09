// #include "astar_planner_ros.h"
// #include "astar_planner.h"
// #include "pluginlib/class_list_macros.hpp"
// #include <boost/thread.hpp>
// #include <nav2_costmap_2d/costmap_2d.hpp>
// #include <nav2_costmap_2d/cost_values.hpp>
// #include <nav_msgs/srv/get_plan.hpp>
// #include <string>
// PLUGINLIB_EXPORT_CLASS(astar_global_planner::AstarGlobalPlannerRos, nav2_core::GlobalPlanner)

// using namespace std;

// namespace astar_global_planner{
//     /*将costmap的四个边的全部栅格都设置LEFHAL_OBSTACLE*/
//     void AstarGlobalPlannerRos::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value)
//     {
//         unsigned char* pc = costarr;
//         for (int i = 0; i < nx; i++)
//             *pc++ = value;
//         pc = costarr + (ny - 1) * nx;
//         for (int i = 0; i < nx; i++)
//             *pc++ = value;
//         pc = costarr;
//         for (int i = 0; i < ny; i++, pc += nx)
//             *pc = value;
//         pc = costarr + nx - 1;
//         for (int i = 0; i < ny; i++, pc += nx)
//             *pc = value;
//     }

//     AstarGlobalPlannerRos::AstarGlobalPlannerRos() 
//             :costmap_(NULL), initialized_(false), allow_unknown_(true),planner_(NULL){
//                 }
//     AstarGlobalPlannerRos::AstarGlobalPlannerRos(std::string name, nav2_costmap_2d::Costmap2DROS* costmap_ros_) 
//             :costmap_(NULL), initialized_(false), allow_unknown_(true),planner_(NULL)
//     {
//         initialize(name, costmap_ros_);
//     }
//     AstarGlobalPlannerRos::AstarGlobalPlannerRos(std::string name, nav2_costmap_2d::Costmap2D* costmap, std::string global_frame) :
//                 costmap_(NULL), initialized_(false), allow_unknown_(true),planner_(NULL)
//     {
//         initialize(name, costmap, global_frame);
//     }

//     void AstarGlobalPlannerRos::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros_)
//     {
//         initialize(name, costmap_ros_->getCostmap(), costmap_ros_->getGlobalFrameID());
//     }

//     // 这里主要是对参数进行初始化，在MoveBase中首先被调用。这里先用参数传入的costmap对地图进行初始化
//     void AstarGlobalPlannerRos::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
//     {
//         if(!initialized_)
//         {
//             costmap_ = costmap;
//             frame_id_ = global_frame;
//             conver_offset = 0.5;
//             // 对成员类AstarGlobalPlanner初始化，这个类将完成全局规划实际计算
//             // 指向路径规划类实例，传入参数为地图大小，已经对地图进行过一次setArr了
//             planner_ = boost::shared_ptr<AstarGlobalPlanner>(new AstarGlobalPlanner(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));
           
//             //创建全局规划器名称下的句柄
//             ros::NodeHandle private_nh("~/" + name); // node_name/name 即move_base_node/name
//             // 发布全局路径规划器 名称/plan话题
//             plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

//             /*开始加载参数服务器上的参数*/
//             // 获取参数服务器上的参数，如果没有，就是用默认值
//             private_nh.param("visualize_potential", visualize_potential_, false);
//             // 如果要将potential array可视化，则发布节点名称下的/potential话题，需要的用户可以订阅
//             //if (visualize_potential_)
//             //{
//             //    potarr_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("potential", 1);
//             //} 
            
//             //继续从参数服务器上获取参数
//             private_nh.param("allow_unknown", allow_unknown_, true);
//             private_nh.param("planner_window_x", planner_window_x_, 0.0);
//             private_nh.param("planner_window_y", planner_window_y_, 0.0);
//             private_nh.param("default_tolerance", default_tolerance_, 0.0);
//             private_nh.param("outline_map",outline_map_, true);

//             // 发布make_plan的服务
//             make_plan_srv = private_nh.advertiseService("make_plan", &AstarGlobalPlannerRos::makePlanService, this);

//             initialized_ = true;

//         }
//         else
//         {
//             ROS_WARN("This global planner has been initialized.......");
//         }
//     }
//     bool AstarGlobalPlannerRos::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
//     {
//         return makePlan(start, goal, default_tolerance_, plan);
//     }

//     // makePlan是在Movebase中对全局规划器调用的函数，它是AstarGlobalPlannerROS类的重点函数，负责调用包括AstarGlobalPlanner类成员在内的函数完成实际计算，控制着全局规划的整个流程。它的输入中最重要的是当前和目标的位置。
//     bool AstarGlobalPlannerRos::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
//     {
//         //由于在move_base中使用了多线程进行规划，为了线程安全，在函数的一开始对信号量mutex_加锁。
//         boost::mutex::scoped_lock lock(mutex_);
//         // 检查是否进行了初始化
//         if (!initialized_)
//         {
//             ROS_ERROR("This planner has not been initialized yet, but it is being used, please initialize it");
//             return false;
//         }
//         // 以防万一 清理plan
//         plan.clear();

//         //创建一个节点句柄
//         ros::NodeHandle n;
        
//         //确保收到的目标和当前位姿都是基于当前的global frame
//         if(goal.header.frame_id != frame_id_){
//             ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
//                       frame_id_.c_str(), goal.header.frame_id.c_str());
//             return false;
//         }

//         if(start.header.frame_id != frame_id_){
//             ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
//                       frame_id_.c_str(), start.header.frame_id.c_str());
//             return false;
//         }
//         /*接下来计算起始点坐标在代价地图中的坐标索引，并用临时的数组map_start记录它们。
//         坐标索引是通过代价地图对象的worldToMap接口获得，这里的wx和wy是两个输入参数， 记录了物理世界中连续的位置坐标; mx和my则是输出参数，输出离散的删格索引。如果不能成功获取删格索引，则报错退出。*/
//         unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
//         int start_x, start_y, goal_x, goal_y;

//         // 起始位姿wx, wy
//         double wx = start.pose.position.x;
//         double wy = start.pose.position.y;
//         //将start转到地图的cell表达坐标
//         if (!costmap_->worldToMap(wx,wy,start_x_i,start_y_i))
//         {
//             ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
//             return false;
//         }
//         else
//         {
//             worldToMap(wx, wy, start_x, start_y);
//         }
//         ROS_INFO("start_x is %f, start_y is %f", wx, wy);
//         //ROS_WARN("start_x_cell in costmap is %d, start_y_cell in costmap is %d", start_x_i, start_y_i);
//         ROS_INFO("start_x_cell is %d, start_y_cell is %d", start_x, start_y);
        
//         //ROS_INFO("origin x %f", costmap_->getOriginX());
//         //ROS_INFO("origin y %f", costmap_->getOriginY());
//         // 获取global系下的目标位置
//         wx = goal.pose.position.x;
//         wy = goal.pose.position.y;
//         // 同样地将目标点也转到map表达
//         if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
//         ROS_WARN_THROTTLE(1.0,
//                 "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
//         return false;
//         }
//         else
//         {
//             worldToMap(wx, wy, goal_x, goal_y);
//         }
        
//         //初始化地图的代价值
//         //planner_->InitMap(costmap_->getCharMap(), true);


//         /*测试初始化地图结果*/
//         /*
//         int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
//         unsigned char* cmpointer = costmap_->getCharMap();
//         int num_information = 0;
//         int num_free = 0;
//         int num_obs = 0;
//         int num_in_obs = 0;
//         int num_other = 0;
//         for (int k=0; k<(nx*ny); k++)
//         {
//             COSTTYPE v = *(cmpointer+k);
//             if (v==costmap_2d::NO_INFORMATION)
//             {
//                 num_information++;
//             //std::cout<< "cost of cell" << v <<std::endl;
//             }
//             else if (v==costmap_2d::FREE_SPACE)
//             {
//                 num_free++;
//             }
//             else if (v==costmap_2d::LETHAL_OBSTACLE)
//             {
//                 num_obs++;
//             }
//             else if (v==costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
//             {
//                 num_in_obs++;
//             }
//             else 
//             {
//                 num_other++;
//             }
//         }
//         ROS_WARN("the number of cell in no information %d", num_information);
//         ROS_WARN("the number of cell in free space %d", num_free); //由于inflation_radius膨胀半径太大，将free space的空间全部挤压了，都变成了指数下降的值
//         ROS_WARN("the number of cell in lethal obstacle %d", num_obs);
//         ROS_WARN("the number of cell in inscribed inflation obstacle %d", num_in_obs);
//         ROS_WARN("the number of cell in other %d", num_other);
//         */
//         /*
//         int test_x = start_x_i;
//         int test_y = start_y_i;
//         int test_index = planner_->CellToIndex(test_x, test_y);
//         COSTTYPE cost = costmap_->getCost(test_x, test_y);
//         COSTTYPE cost_1 = cmpointer[test_index];
//         COSTTYPE cost_2 = *(cmpointer+test_index);
//         ROS_WARN("the cost value is %d", cost);
//         ROS_WARN("the cost value 1 is %d", cost_1);
//         ROS_WARN("the cost value 2 is %d", cost_2);
//         // 说明我的世界坐标转地图坐标的函数出问题了，导致我的cost取值不对
//         */

//         //clear the starting cell within the costmap because we know it can't be an obstacle
//         // 将开始点设置为FREE_SPACE
//         clearRobotCell(start, start_x, start_y);
        
//         /*获取costmap的各种参数以及函数*/
//         int nx_map = costmap_->getSizeInCellsX(), ny_map = costmap_->getSizeInCellsY(); // 长和宽都是384(cell)    
    
        
//         // 将costmap的四个边的全部栅格设置为LETHAL_OBSTACLE
//         if (outline_map_)
//         {
//             outlineMap(costmap_->getCharMap(), nx_map, ny_map, costmap_2d::LETHAL_OBSTACLE);
//         }
//         //std::cout<< "before x of termination" << planner_->terminatePtr->NodeX<< std::endl;
//         bool found_path = planner_->calcAstarPath(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y);
//         /*closelist中的起点*/
//         AstarNode firstNode = (planner_->closelist_)[0];
//         int X_first = firstNode.NodeX;
//         int Y_first = firstNode.NodeY;
//         ROS_WARN("x first %d and y first %d", X_first, Y_first);
//         /*closelist中的终点*/
//         AstarNode finalNode = (planner_->closelist_).back();
//         int X_final = finalNode.NodeX;
//         int Y_final = finalNode.NodeY;
//         ROS_WARN("x final %d and y final %d", X_final, Y_final);
//         ROS_WARN("if find an available path %d", found_path); //0--未找到路径, 1--找到路径

//         int num = (planner_->closelist_).size();
//         ROS_WARN("the number of node in closelist %d", num);

//         if (found_path)
//         {
//             //提取路径
//             //因为openlist和closelist都是节点对象，在找到父节点的xy坐标后，必须在list中找到相对应的节点对象，才能找到上一个的父节点的xy坐标，如果openlist和closelist都是指针就好办了
//             //所有的node应该都在closelist中
//             std::vector<std::pair<int, int>> path_ = planner_->getAstarPath(start_x, start_y, goal_x, goal_y);
//             ROS_WARN("x begin in path %d and y begin in path %d", path_[0].first, path_[0].second);
//             ROS_WARN("x end in path %d and y end in path %d", path_.back().first, path_.back().second);
//             ROS_WARN("numbe of node in path %ld", path_.size());
//             std::string global_frame_ = frame_id_;
//             ros::Time plan_time = ros::Time::now();
//             for (int i = 0; i<path_.size(); i++)
//             {
//                 geometry_msgs::PoseStamped current_pose;
//                 double world_x; 
//                 double world_y;
//                 mapToWorld(path_[i].first, path_[i].second,world_x, world_y);
//                 current_pose.pose.position.x = world_x;
//                 current_pose.pose.position.y = world_y;
//                 current_pose.pose.position.z = 0.0;
//                 current_pose.pose.orientation.x = 0.0;
//                 current_pose.pose.orientation.y = 0.0;
//                 current_pose.pose.orientation.z = 0.0;
//                 current_pose.pose.orientation.w = 1.0;
//                 current_pose.header.stamp = plan_time;
//                 current_pose.header.frame_id = global_frame_;
//                 plan.push_back(current_pose);
//             }
//         }
//         else
//         {
//             ROS_ERROR("Failed to get a plan..........");
//             return false;
//         }
//         publishPlan(plan);
//         return !plan.empty();
//     }

//     void AstarGlobalPlannerRos::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my)
//     {
//         if (!initialized_)
//         {
//             ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
//             return;
//         }
//         costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
//     }

//     bool AstarGlobalPlannerRos::worldToMap(double wx, double wy, int& mx, int& my)
//     {
//         double origin_x = costmap_->getOriginX();
//         double origin_y = costmap_->getOriginY();
//         double resolution = costmap_->getResolution();

//         if (wx < origin_x || wy < origin_y)
//         {
//             return false;
//         }
//         mx = (int)((wx - origin_x) / resolution);
//         my = (int)((wy - origin_y) / resolution);
//         //mx = (wx - origin_x) / resolution - convert_offset_;
//         if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
//             return true;

//         return false;
//     }

//     void AstarGlobalPlannerRos::mapToWorld(double mx, double my, double& wx, double& wy)
//     {
//         wx = costmap_->getOriginX() + (mx+conver_offset) * costmap_->getResolution();
//         wy = costmap_->getOriginY() + (my+conver_offset) * costmap_->getResolution();
//     }

//     void AstarGlobalPlannerRos::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
//     {
//         if (!initialized_) 
//         {
//         ROS_ERROR(
//                 "This planner has not been initialized yet, but it is being used, please call initialize() before use");
//         return;
//         }
//          //create a message for the plan
//         nav_msgs::Path gui_path;
//         gui_path.poses.resize(path.size());

//         gui_path.header.frame_id = frame_id_;
//         gui_path.header.stamp = ros::Time::now();

//         // Extract the plan in world co-ordinates, we assume the path is all in the same frame
//         for (unsigned int i = 0; i < path.size(); i++) {
//             gui_path.poses[i] = path[i];
//         }

//         plan_pub_.publish(gui_path);
//     }

//     bool AstarGlobalPlannerRos::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) 
//     {
//         makePlan(req.start, req.goal, resp.plan.poses);

//         resp.plan.header.stamp = ros::Time::now();
//         resp.plan.header.frame_id = frame_id_;

//         return true;
//     }
// };