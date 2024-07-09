#include "rclcpp/rclcpp.hpp"
#include "astar_planner.h"

#include <vector>
#include <list>
#include <math.h>
using namespace std;

///////////////////////////////////////////////////////////////////////////
const int kCost1 = 10; //直移一格消耗
const int kCost2 = 17; //斜移一格消耗

struct Point
{
	int x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
	int F, G, H; //F=G+H
	Point *parent; //parent的坐标，这里没有用指针，从而简化代码
	Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)  //变量初始化
	{
	}
};

class Astar
{
public:
    //构造函数，输入为_maze的地址
	void InitAstar(std::vector<std::vector<int>> &_maze);
    //路径点list
	std::list<Point *> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
private:
    //寻找路径的函数，返回的是点的指针
	Point *findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
    //获取周围点
    std::vector<Point *> getSurroundPoints(const Point *point, bool isIgnoreCorner) const;
	//判断某点是否可以用于下一步判断
    bool isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const; 
	 //判断开启/关闭列表中是否包含某点
    Point *isInList(const std::list<Point *> &list, const Point *point) const;
    //从开启列表中返回F值最小的节点
	Point *getLeastFpoint(); 
	//计算FGH值
	int calcG(Point *temp_start, Point *point);
	int calcH(Point *point, Point *end);
	int calcF(Point *point);
private:
	std::vector<std::vector<int>> maze;
	std::list<Point *> openList;  //开启列表
	std::list<Point *> closeList; //关闭列表
public:
	bool maze_flag = true; //加载地图的标志位，省得多次加载浪费资源
};

void Astar::InitAstar(std::vector<std::vector<int>> &_maze){
	maze = _maze;
}

Point *Astar::getLeastFpoint() //返回最小F的点
{
	if (!openList.empty())
	{
		auto resPoint = openList.front();  //返回第一个元素
		for (auto &point : openList){
			if (point->F<resPoint->F)
			resPoint = point;
		}
		return resPoint;
	}
	return NULL;
}
Point *Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	openList.push_back(new Point(startPoint.x, startPoint.y)); //置入起点,拷贝开辟一个节点，内外隔离
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //找到F值最小的点
		//ROS_INFO("%d,%d",curPoint->x,curPoint->y);
		openList.remove(curPoint); //从开启列表中删除
		closeList.push_back(curPoint); //放到关闭列表
		//1,找到当前周围八个格中可以通过的格子
		//ROS_INFO("改找周围点了");
		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
		for (auto &target : surroundPoints)
		{
			//ROS_INFO("开始判断周围点了");
			//2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
			if (!isInList(openList, target))
			{
				//BROS_INFO("判断每个目标点");
				target->parent = curPoint; //给子节点放父节点
 
				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);
 
				openList.push_back(target);
			}
			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
			else
			{
				int tempG = calcG(curPoint, target);
				if (tempG<target->G)
				{
					target->parent = curPoint;
 
					target->G = tempG;
					target->F = calcF(target);
				}
			}
			{   //如果到这一步，就说明已经结束了
				Point *resPoint = isInList(openList, &endPoint);
				if (resPoint)
					return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
			}
		}
	}
	return NULL;
}
 
std::list<Point *> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	//ROS_INFO("开始坐标:(%d,%d),结束坐标:(%d,%d)",startPoint.x,startPoint.y,endPoint.x,endPoint.y);
	Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
	//ROS_INFO("找到路径了");
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
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"获取路径成功");
	return path;
}
 
Point *Astar::isInList(const std::list<Point *> &list, const Point *point) const
{
	//判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
	for (auto p : list){
		if (p->x == point->x&&p->y == point->y)
		return p;
	}
	return NULL;
}
 
bool Astar::isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const
{
	//ROS_INFO("判断是否可以到达");
	//判断是否可以到达
	if (target->x<0 || target->x>maze.size()-1    //x不在地图边界内
		|| target->y<0 || target->y>maze[0].size() - 1 //y不在地图边界内
		|| maze[target->x][target->y] == 100	//障碍物
		|| (target->x == point->x&&target->y == point->y)		//与当前节点重合
		|| isInList(closeList, target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
	{
		return false;
	}
	else
	{
		//ROS_INFO("可以到达");
		if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //非斜角可以
			{
				//ROS_INFO("非斜角");
				return true;
			}
		else
		{
			//斜对角要判断是否绊住，意思就是我能不能从直线走到斜对角去
			if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0){
				//ROS_INFO("拌不住");
				return true;
			}
				
			else{
				//ROS_INFO("绊住了");
				return isIgnoreCorner; //到这就说明是真的被绊住了，那就看用不用考虑会被绊住的条件
			}
				
		}
	}
}
 
std::vector<Point *> Astar::getSurroundPoints(const Point *point, bool isIgnoreCorner) const
{
	std::vector<Point *> surroundPoints;
	for (int x = point->x - 1; x <= point->x + 1; x++){
		for (int y = point->y - 1; y <= point->y + 1; y++){
			if (isCanreach(point, new Point(x, y), isIgnoreCorner))
				{
					surroundPoints.push_back(new Point(x, y));
					//ROS_INFO("在放点");
				}
		}
	}
	return surroundPoints;
}

int Astar::calcG(Point *temp_start, Point *point){
	//如果x，y绝对值加起来是0那么就是一个点，如果是差1，那么就是差一格，如果不是1那就是斜着走的
	int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2; 
	//如果是初始节点，则其父节点是空，父节点的G就为0
	int parentG = point->parent == NULL ? 0 : point->parent->G; 
	return parentG + extraG;
}

int Astar::calcH(Point *point, Point *end){
	//用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法，没深入研究^_^
	//return sqrt((double)(end->x - point->x)*(double)(end->x - point->x) + (double)(end->y - point->y)*(double)(end->y - point->y))*kCost1;
	return (abs(end->x - point->x)+abs(end->y - point->y)) * kCost1;
}
 
int Astar::calcF(Point *point){
	return point->G + point->H;
}
///////////////////////////////////////////////////////////////////////////
float init_pose_point[3]={0};
float goal_pose_point[3]={0};
// int *map_data_p; //地图数据指针

class MapClass{
    public:
        int origin_x = 0;
        int origin_y = 0;
        float resolution = 0;
        int width = 0;
        int height = 0;
        vector<vector<int>> map_data;
    public:
        void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
};


void MapClass::MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    origin_x = msg->info.origin.position.x;
    origin_y = msg->info.origin.position.y;
    resolution = msg->info.resolution; //像素：迷/像素
    width = msg->info.width;
    height = msg->info.height;
	// for(int i=0;i<height;i++) map_data.push_back(vector<int>());
	// for(int i=0;i<height;i++){
	// 	for(int j=0;j<width;j++){
	// 		map_data[j].push_back(msg.data[i*width + j]);
	// 		if(msg.data[i*width + j]==100)
	// 	}
	// } 
	for(int i=0;i<width;i++) map_data.push_back(vector<int>());
	for(int i=0;i<width;i++){
		for(int j=0;j<height;j++){
			map_data[i].push_back(msg->data[j*width+i]);
		}
	}
}
void InitPoseCallback(const geometry_msgs::msg::PoseWithCovariance msg){
	init_pose_point[0] = 1;
    init_pose_point[1] = msg.pose.position.x;
    init_pose_point[2] = msg.pose.position.y;

}
void GoalPoseCallback(const geometry_msgs::msg::PoseStamped msg){
	goal_pose_point[0] = 1;
    goal_pose_point[1] = msg.pose.position.x;
    goal_pose_point[2] = msg.pose.position.y;
}



int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    rclcpp::init(argc, argv);
    auto node =std::make_shared<rclcpp::Node>("astar_planning_node");
    MapClass mapclass;//创建地图类
	Astar astar;
	bool IsGetPath =false;
	list<Point *> path_copy; 
    // 订阅地图消息
    auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            mapclass.MapCallback(msg);
        });
	auto pose_sub =node ->create_subscription<geometry_msgs::msg::Pose>("/initialpose",10,
		[&](const geometry_msgs::msg::Pose::SharedPtr pose)
		{
			InitPoseCallback(pose);
		});
    // 创建路径发布者
    auto path_pub = node->create_publisher<nav_msgs::msg::Path>("path_Astar", 10);

    // 创建路径消息
    nav_msgs::msg::Path pathforpub;
    pathforpub.header.frame_id = "map";

    // 设置ROS 2的循环频率
    rclcpp::Rate r(1);

    while (rclcpp::ok())
    {
        // 假设有起点和终点坐标
        Point start(0, 0);
        Point end(10, 10);
		
        // 设置地图数据到A*算法中
        if (mapclass.map_data.empty())
        {
            RCLCPP_INFO(node->get_logger(), "Waiting for map data...");
            rclcpp::spin_some(node);
            continue;
        }

        astar.InitAstar(mapclass.map_data);

        // 调用A*算法获取路径
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Planning path...");
        std::list<Point *> path = astar.GetPath(start, end, true);

        // 将路径信息填充到pathforpub中
        pathforpub.poses.clear();
        for (auto *point : path)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = mapclass.origin_x + point->x * mapclass.resolution;
            pose.pose.position.y = mapclass.origin_y + point->y * mapclass.resolution;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pathforpub.poses.push_back(pose);
        }

        // 发布路径消息
        pathforpub.header.stamp = node->now();
        path_pub->publish(pathforpub);

        // 执行ROS 2循环处理
        rclcpp::spin_some(node);
        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}