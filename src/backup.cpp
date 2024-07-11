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

using namespace std;
 
const int kCost1 = 10; //直移一格消耗
const int kCost2 = 14; //斜移一格消耗
 


struct Point
{
	int x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
	int F, G, H; //F=G+H
	Point *parent; //parent的坐标，这里没有用指针，从而简化代码
	Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)  //变量初始化
	{
	}
};

// multiset自定义排序规则
class cmp{
public:
	bool operator() (const Point * A, const Point * B) const
	{
		if(A->F != B->F){
			return A->F < B->F;
		}
		else 
			return A->F > B->F;
	}
};
 
class Astar
{
public:
	void InitAstar(std::vector<std::vector<int>> &_maze);
	std::list<Point *> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
 
private:
	Point *findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
	std::vector<Point *> getSurroundPoints(const Point *point, bool isIgnoreCorner) const;
	bool isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断
	// Point *isInList(const std::list<Point *> &list, const Point *point) const; //判断开启/关闭列表中是否包含某点

	Point *isInList(const std::multiset<Point *, cmp> &list, const Point *point) const; //判断开启/关闭列表中是否包含某点
	// Point *getLeastFpoint(); //从开启列表中返回F值最小的节点
	//计算FGH值
	int calcG(Point *temp_start, Point *point);
	int calcH(Point *point, Point *end);
	int calcF(Point *point);
public:
	std::vector<std::vector<int>> maze;
	// std::list<Point *> openList;  //开启列表
	std::multiset<Point *, cmp> openList;
	std::multiset<Point *, cmp> closeList; //关闭列表

public:
	bool maze_flag = true; //加载地图的标志位，省得多次加载浪费资源

};


void Astar::InitAstar(std::vector<std::vector<int>> &_maze)
{
	maze = _maze;
}

int Astar::calcG(Point *temp_start, Point *point)
{
	int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2;
	int parentG = point->parent == NULL ? 0 : point->parent->G; //如果是初始节点，则其父节点是空
	return parentG + extraG;
}

// point是当前点，end是终点
int Astar::calcH(Point *point, Point *end)
{
	//用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法，没深入研究^_^
	return sqrt((double)(end->x - point->x)*(double)(end->x - point->x) + (double)(end->y - point->y)*(double)(end->y - point->y))*kCost1;

	// 用曼哈顿距离进行计算
	// return abs(end->x - point->x) + abs(end->y - point->y);
}

int Astar::calcF(Point *point)
{
	return point->G + point->H;
}
 
// Point *Astar::getLeastFpoint()
// {
// 	if (!openList.empty())
// 	{
// 		auto resPoint = openList.front();
// 		for (auto &point : openList)
// 			if (point->F<resPoint->F)
// 				resPoint = point;
// 		return resPoint;
// 	}
// 	return NULL;
// }
 
Point *Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	// openList.push_back(new Point(startPoint.x, startPoint.y)); //置入起点,拷贝开辟一个节点，内外隔离
	openList.insert(new Point(startPoint.x, startPoint.y)); //置入起点,拷贝开辟一个节点，内外隔离
	cout << startPoint.x << ", " << startPoint.y << endl;
	int count = 0;
	while (!openList.empty())
	{

		auto curPoint = openList.begin();

		openList.erase(curPoint); //从开启列表中删除
		closeList.insert(*curPoint); //放到关闭列表
		

				//1,找到当前周围八个格中可以通过的格子
		auto surroundPoints = getSurroundPoints(*curPoint, isIgnoreCorner);
		for (auto &target : surroundPoints)
		{
			//2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
			if (!isInList(openList, target))
			{
				target->parent = *curPoint;

				// 计算部分可能计算慢
				target->G = calcG(*curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);
 
				openList.insert(target);
				
			}
			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
			else
			{
				int tempG = calcG(*curPoint, target);
				if (tempG<target->G)
				{
					target->parent = *curPoint;
 
					target->G = tempG;
					target->F = calcF(target);
				}
			}
			Point *resPoint = isInList(openList, &endPoint);
			if (resPoint)
				return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
		}

	}
 
	return NULL;
}


std::list<Point *> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
	std::list<Point *> path;

	//返回路径，如果没找到路径，返回空链表
	while (result)
	{
		path.push_front(result);
		result = result->parent;
	}
 
	// 清空临时开闭列表，防止重复执行GetPath导致结果异常
	openList.clear();
	closeList.clear();
 
	return path;
}

 

Point *Astar::isInList(const std::multiset<Point *, cmp> &list, const Point *point) const
{
	//判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
	for (auto p : list)
		if (p->x == point->x&&p->y == point->y)
			return p;
	return NULL;
}
 
bool Astar::isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const
{

	if (target->x<0 || target->x>maze.size() - 1
		|| target->y<0 || target->y>maze[0].size() - 1
		|| maze[target->x][target->y] == 100
		|| target->x == point->x&&target->y == point->y
		|| isInList(closeList, target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
		{
			return false;
		}
	else
	{
		if (abs(point->x - target->x) + abs(point->y - target->y) == 1) {
			if(maze[target->x][target->y] == 100){
				cout << "该点是100的点，但return了true1" << endl;
			}	
			return true;
		}//非斜角可以
		else
		{
			//斜对角要判断是否绊住
			if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0){
				if(maze[target->x][target->y] == 100){
					cout << "该点是100的点，但return了true2" << endl;
					return true;
				}	
			}
				
			else
				return isIgnoreCorner;
		}
	}
}
 
std::vector<Point *> Astar::getSurroundPoints(const Point *point, bool isIgnoreCorner) const
{
	std::vector<Point *> surroundPoints;
 
	for (int x = point->x - 1; x <= point->x + 1; x++){
		for (int y = point->y - 1; y <= point->y + 1; y++){
			if (isCanreach(point, new Point(x, y), isIgnoreCorner)){
				surroundPoints.push_back(new Point(x, y));
			}
				
		}
	}
	return surroundPoints;
		

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
		timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&astar_planning::timer_callback, this));

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
		for(int i=0;i<width;i++) map_data.push_back(vector<int>());
		for(int i=0;i<width;i++){
			for(int j=0;j<height;j++){
				map_data[i].push_back(msg.data[j*width+i]);
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
		Astar astar;
		bool IsGetPath =false;
		list<Point *> path_copy;
		nav_msgs::msg::Path pathforpub;
		pathforpub.header.frame_id = "map";
		pathforpub.header.stamp = rclcpp::Clock().now();
		if(height>0 && width> 0 && astar.maze_flag)
		{
			astar.maze_flag = false;
			astar.InitAstar(map_data);

		}

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
			Point start((int)init_pose_point[1],(int)init_pose_point[2]);
			Point end((int)goal_pose_point[1],(int)goal_pose_point[2]);
			// ROS_INFO("%d,%d",start.x,start.y);
			// ROS_INFO("%d,%d",end.x,end.y);
			//A星算法寻找路径
			list<Point *> path = astar.GetPath(start, end, false); //这个其实是反过来的
			pathforpub.poses.clear();
			for (auto p: path)  {
				geometry_msgs::msg::PoseStamped this_pose_stamped;
				this_pose_stamped.header.frame_id="map";
				pathforpub.header.stamp = rclcpp::Clock().now();
				this_pose_stamped.pose.position.x = p->x*resolution;
				this_pose_stamped.pose.position.y = p->y*resolution;

				this_pose_stamped.pose.orientation.x = 0;
				this_pose_stamped.pose.orientation.y = 0;
				this_pose_stamped.pose.orientation.z = 0;
				this_pose_stamped.pose.orientation.w = 0; //直接让四元数全为0了，省得
				pathforpub.poses.push_back(this_pose_stamped);
				result_data_publisher->publish(pathforpub);
			}
			
				//path_pub.publish(pathforpub);
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

