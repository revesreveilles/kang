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
#include <fstream>
#include <iostream>
#include <set>
#include <algorithm>
#include <math.h>

using namespace std;
 
/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/


#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>
#include <algorithm>
#include <math.h>

namespace AStar
{
    struct Vec2i
    {
        int x, y;

        bool operator == (const Vec2i& coordinates_);
        friend Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_) {
            return{ left_.x + right_.x, left_.y + right_.y };
        }
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    struct Node
    {
        uint G, H;
        Vec2i coordinates;
        Node *parent;

        Node(Vec2i coord_, Node *parent_ = nullptr);
        uint getScore();
    };

    using NodeSet = std::vector<Node*>;

    class Generator
    {
        bool detectCollision(Vec2i coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void setWorldSize(Vec2i worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(Vec2i source_, Vec2i target_);
        void addCollision(Vec2i coordinates_);
        void removeCollision(Vec2i coordinates_);
        void clearCollisions();

    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        Vec2i worldSize;
		// 开销,斜线8,直线4
        uint directions;
    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint manhattan(Vec2i source_, Vec2i target_);
        static uint euclidean(Vec2i source_, Vec2i target_);
        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__



using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;
		// 在current点附近找F最小的节点
        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }
		// 如果openset中有终点,那么直接退出
        if (current->coordinates == target_) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);
        for (uint i = 0; i < directions; ++i) {
			// 查找本节点周围可达节点
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {	
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}

/*------------------------------------以上是A*算法实现流程---------------------------------------------------*/

float init_pose_point[3]={0};
float goal_pose_point[3]={0};
int origin_x = 0;
int origin_y = 0;
float resolution = 0;
int width = 0;
int height = 0;
vector<vector<int>> map_data;

AStar::Generator generator;



class astar_planning : public rclcpp::Node{
public:
	astar_planning(string name):Node(name){
		RCLCPP_INFO(this->get_logger(), "%s节点已经启动", name.c_str());
  		// 地图数据订阅者
		map_data_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10000, 
  		bind(&astar_planning::MapCallback, this, placeholders::_1));
		// 地图起点数据订阅者
		init_data_subscriber = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10000, 
  		std::bind(&astar_planning::init_data_callback, this, placeholders::_1));
		// 地图终点数据订阅者
		goal_data_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10000, 
		std::bind(&astar_planning::data_callback, this, placeholders::_1));

		// 地图数据发布节点
		result_data_publisher = this->create_publisher<nav_msgs::msg::Path>("path", 200000);
		timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&astar_planning::timer_callback, this));

	}
private:
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_data_subscriber;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_data_subscriber;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_data_subscriber;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr result_data_publisher;
	rclcpp::TimerBase::SharedPtr timer;
	void MapCallback(const nav_msgs::msg::OccupancyGrid msg){
		RCLCPP_INFO(this->get_logger(), "地图订阅获取到数据");
		
		origin_x = msg.info.origin.position.x;
		origin_y = msg.info.origin.position.y;
		resolution = msg.info.resolution; //像素：迷/像素
		width = msg.info.width;
		height = msg.info.height;
		// for(int i=0;i<height;i++) map_data.push_back(vector<int>());
		// for(int i=0;i<height;i++){
		// 	for(int j=0;j<width;j++){
		// 		map_data[j].push_back(msg.data[i*width + j]);
		// 		if(msg.data[i*width + j]==100)
		// 	}
		// } //祭奠我傻逼的获取地图数据
        AStar::Vec2i coll;
		for(int i=0;i<width;i++) map_data.push_back(vector<int>());
		for(int i=0;i<width;i++){
			for(int j=0;j<height;j++){
				map_data[i].push_back(msg.data[j*width+i]);
                if(map_data[i][j] == 100){
                    coll.x = i;
                    coll.y = j;
                    generator.addCollision(coll);
                }
				// outFile << map_data[i][j] << "  ";
			}
			// outFile << endl;
		}
		
	}

	// 地图起点数据订阅者回调函数
	void init_data_callback(const geometry_msgs::msg::PoseWithCovarianceStamped msg){
		RCLCPP_INFO(this->get_logger(), "起点数据已经调用");
		init_pose_point[0] = 1;
		init_pose_point[1] = msg.pose.pose.position.x;
		init_pose_point[2] = msg.pose.pose.position.y;
	}
	// 地图终点数据订阅者
	void data_callback(const geometry_msgs::msg::PoseStamped msg){
		RCLCPP_INFO(this->get_logger(), "终点数据已经调用");
		goal_pose_point[0] = 1;
		goal_pose_point[1] = msg.pose.position.x;
		goal_pose_point[2] = msg.pose.position.y;

	}
	// 定时发布数据
	void timer_callback(){
        // AStar::Generator generator;
        // if(height>0 && width> 0 && astar.maze_flag)
		// {
		// 	astar.maze_flag = false;
		// 	astar.InitAstar(map_data);
		// }
        generator.setWorldSize({width, height});
        generator.setHeuristic(AStar::Heuristic::euclidean);
        generator.setDiagonalMovement(true);
        // AStar::Vec2i coll;

        
        
		bool IsGetPath =false;
		// list<Point *> path_copy;
		nav_msgs::msg::Path pathforpub;
		pathforpub.header.frame_id = "map";
		pathforpub.header.stamp = rclcpp::Clock().now();


		if(init_pose_point[0]&&goal_pose_point[0]){
			
			init_pose_point[0] = 0;
			init_pose_point[1] = init_pose_point[1]/resolution;
			init_pose_point[2] = init_pose_point[2]/resolution;
			goal_pose_point[0] = 0;
			goal_pose_point[1] = goal_pose_point[1]/resolution;
			goal_pose_point[2] = goal_pose_point[2]/resolution;
			// RCLCPP_INFO(this->get_logger(), "终点数据为 x: %f, y:%f", goal_pose_point[1], goal_pose_point[2]);
			// RCLCPP_INFO(this->get_logger(), "起点数据为 x: %f, y:%f", init_pose_point[1], init_pose_point[2]);

			// ROS_INFO("%f,%f",init_pose_point[1],init_pose_point[2]);
			// ROS_INFO("%f,%f",goal_pose_point[1],goal_pose_point[2]);

            std::cout << "Generate path ... \n";

            auto path = generator.findPath({(int)init_pose_point[1],(int)init_pose_point[2]}, {(int)goal_pose_point[1],(int)goal_pose_point[2]});
			pathforpub.poses.clear();

            for(auto& p : path) {
                // std::cout << coordinate.x << " " << coordinate.y << "\n";
                geometry_msgs::msg::PoseStamped this_pose_stamped;
				this_pose_stamped.header.frame_id="map";
                
				pathforpub.header.stamp = rclcpp::Clock().now();
				this_pose_stamped.pose.position.x = p.x*resolution;
				this_pose_stamped.pose.position.y = p.y*resolution;

				this_pose_stamped.pose.orientation.x = 0;
				this_pose_stamped.pose.orientation.y = 0;
				this_pose_stamped.pose.orientation.z = 0;
				this_pose_stamped.pose.orientation.w = 0; //直接让四元数全为0了，省得
				pathforpub.poses.push_back(this_pose_stamped);
				result_data_publisher->publish(pathforpub);
            }
			
		}
		// result_data_publisher->publish(pathforpub);
	}
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto node = make_shared<astar_planning>("astar_planning");
	rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

