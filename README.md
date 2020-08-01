# A*
## Part 0 准备工作
- git clone
```sh
mkdir -p your_workspace/src && cd your_workspace/src
git clone https://github.com/Zcyyy/robot_tutorial_01.git
catkin build
```
- 使用rviz打开
在终端执行```rviz```
点击```File``` -> ```open config``` -> 选择 grid_path_searcher/launch/rviz_config/demo.rviz
## Part 1 代码执行流程
- 见文件:src/grid_path_searcheer/src/demo_node.cpp
```sh
int main(int argc, char** argv){
    ……
    //订阅地图信息的回调
    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    //订阅终点信息的回调
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    ……
    //定义了结构体 AstarPathFinder 变量_astar_path_finder,该结构体存储、实现了 Astar 路径规划所需的所有信息和功能
    _astar_path_finder  = new AstarPathFinder();
    _astar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    ……
}
```
- 回调函数rcvPointCloudCallBack
```sh
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    ……
    //将障碍物信息设置到栅格化地图中，为后续路径规划做准备
    _astar_path_finder->setObs(pt.x, pt.y, pt.z);
    ……
    //可视化地图部分
    _grid_map_vis_pub.publish(map_vis);
    ……
}
```
- 回调函数rcvWaypointsCallBack
```sh
void rcvWaypointsCallback(const nav_msgs::Path & wp)
{
  ……
  //获取交互式界面给出的终点坐标
  target_pt << wp.poses[0].pose.position.x,
  wp.poses[0].pose.position.y,
  wp.poses[0].pose.position.z;
  ……
  //输入起点、终点,调用 pathFind 函数
  pathFinding(_start_pt, target_pt);
 }
```
- 路径规划函数 pathFinding
```sh
void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
{
  //使用 A*进行路径搜索
  _astar_path_finder->AstarGraphSearch(start_pt, target_pt);
  //获取规划的路径
  auto grid_path
  ……
  auto grid_path = _astar_path_finder->getPath();
  //可视化结果
  visGridPath (grid_path, false);
  ……
  //为下次规划重置地图
  _astar_path_finder->resetUsedGrids();
}
```
## Part2 涉及类和结构体的简介
- 节点表示:用结构体变量 GridNode 表示,存储了节点的坐标、g(n)、f(n)值、父节点指
针等信息。
```sh
struct GridNode
{     
    int id;        // 1--> open set, -1 --> closed set
    Eigen::Vector3d coord; 
    Eigen::Vector3i dir;   // direction of expanding
    Eigen::Vector3i index;
	
    double gScore, fScore;
    GridNodePtr cameFrom;
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
		id = 0;
		index = _index;
		coord = _coord;
		dir   = Eigen::Vector3i::Zero();

		gScore = inf;
		fScore = inf;
		cameFrom = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};
```
- 父类AstarPathFinder
```sh
class AstarPathFinder
{	
	private:

	protected:
		……
    //open set 实现:用C++ STL 中的 multimap 实现
		std::multimap<double, GridNodePtr> openSet;
    //启发式函数，作业
		double getHeu(GridNodePtr node1, GridNodePtr node2);
    //拓展节点函数，作业
		void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);		

	public:
    //A*搜索函数，作业
		void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
		
};
```


## Part 3 任务详情
- 完成src/grid_path_searcher/Astar_searcher.cpp下的  
```sh
void AstarPathFinder::AstarGetSucc(...);
double AstarPathFinder::getHeu(...);
void AstarPathFinder::AstarGraphSearch(...);
vector<Vector3d> AstarPathFinder::getPath(...);
```
