/***
 * @......................................&&.........................
 * @....................................&&&..........................
 * @.................................&&&&............................
 * @...............................&&&&..............................
 * @.............................&&&&&&..............................
 * @...........................&&&&&&....&&&..&&&&&&&&&&&&&&&........
 * @..................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&..............
 * @................&...&&&&&&&&&&&&&&&&&&&&&&&&&&&&.................
 * @.......................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&.........
 * @...................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&...............
 * @..................&&&   &&&&&&&&&&&&&&&&&&&&&&&&&&&&&............
 * @...............&&&&&@  &&&&&&&&&&..&&&&&&&&&&&&&&&&&&&...........
 * @..............&&&&&&&&&&&&&&&.&&....&&&&&&&&&&&&&..&&&&&.........
 * @..........&&&&&&&&&&&&&&&&&&...&.....&&&&&&&&&&&&&...&&&&........
 * @........&&&&&&&&&&&&&&&&&&&.........&&&&&&&&&&&&&&&....&&&.......
 * @.......&&&&&&&&.....................&&&&&&&&&&&&&&&&.....&&......
 * @........&&&&&.....................&&&&&&&&&&&&&&&&&&.............
 * @..........&...................&&&&&&&&&&&&&&&&&&&&&&&............
 * @................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&............
 * @..................&&&&&&&&&&&&&&&&&&&&&&&&&&&&..&&&&&............
 * @..............&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&....&&&&&............
 * @...........&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&......&&&&............
 * @.........&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&.........&&&&............
 * @.......&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&...........&&&&............
 * @......&&&&&&&&&&&&&&&&&&&...&&&&&&...............&&&.............
 * @.....&&&&&&&&&&&&&&&&............................&&..............
 * @....&&&&&&&&&&&&&&&.................&&...........................
 * @...&&&&&&&&&&&&&&&.....................&&&&......................
 * @...&&&&&&&&&&.&&&........................&&&&&...................
 * @..&&&&&&&&&&&..&&..........................&&&&&&&...............
 * @..&&&&&&&&&&&&...&............&&&.....&&&&...&&&&&&&.............
 * @..&&&&&&&&&&&&&.................&&&.....&&&&&&&&&&&&&&...........
 * @..&&&&&&&&&&&&&&&&..............&&&&&&&&&&&&&&&&&&&&&&&&.........
 * @..&&.&&&&&&&&&&&&&&&&&.........&&&&&&&&&&&&&&&&&&&&&&&&&&&.......
 * @...&&..&&&&&&&&&&&&.........&&&&&&&&&&&&&&&&...&&&&&&&&&&&&......
 * @....&..&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&...........&&&&&&&&.....
 * @.......&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&..............&&&&&&&....
 * @.......&&&&&.&&&&&&&&&&&&&&&&&&..&&&&&&&&...&..........&&&&&&....
 * @........&&&.....&&&&&&&&&&&&&.....&&&&&&&&&&...........&..&&&&...
 * @.......&&&........&&&.&&&&&&&&&.....&&&&&.................&&&&...
 * @.......&&&...............&&&&&&&.......&&&&&&&&............&&&...
 * @........&&...................&&&&&&.........................&&&..
 * @.........&.....................&&&&........................&&....
 * @...............................&&&.......................&&......
 * @................................&&......................&&.......
 * @.................................&&..............................
 * @..................................&..............................
 * @
 * @Author: your name
 * @Date: 2022-11-21 19:45:46        AstarPathFinder()
				{
						g_method = Diagonal; //默认使用对角线距离
						h_method = Diagonal; //默认使用对角线距离

						g_value = 1.0;    //默认权重是1
						h_value = 1.0;    //默认权重是1

						std::cout<<"G-method: "<<g_method<<std::endl;
						std::cout<<"H-method: "<<h_method<<std::endl;

						std::cout<<"G-weight: "<<g_value<<std::endl;
						std::cout<<"H-weight: "<<h_value<<std::endl;

				}i/src/astar_super/src/Astar_searcher.cpp
 * @可以输入预定的版权声明、个性签名、空行等
 */

#include "path_searcher/Astar_searcher.h"

using namespace std;
using namespace Eigen;

// 用于设定一些默认的参数
AstarPathFinder::AstarPathFinder()
{
	// std::cout<<"Astar Config"<<std::endl;

	g_method = Diagonal; // 默认使用对角线距离
	h_method = Diagonal; // 默认使用对角线距离

	g_value = 1.0; // 默认权重是1
	h_value = 1.0; // 默认权重是1

	// std::cout<<"G-method: "<<g_method<<std::endl;
	// std::cout<<"H-method: "<<h_method<<std::endl;

	// std::cout<<"G-weight: "<<g_value<<std::endl;
	// std::cout<<"H-weight: "<<h_value<<std::endl;
}

// 用于传参
void AstarPathFinder::InitParams(ros::NodeHandle &nh,bool _is_use_esdf)
{
	nh.param("astar_weight/g", g_value, 1.0);          // 权重a值
	nh.param("astar_weight/h", h_value, 1.0);          // 权重b值
	nh.param("astar_heuristic/distance", h_method,0); // 0 1 2 3方法 欧氏距离 曼哈顿 切比学夫 对角线
	nh.param("astar_glength/distance",g_method ,0);
	nh.param("astar_search/radius", search_radius, 100.0);
	is_use_esdf = _is_use_esdf;	
}

int AstarPathFinder::getESDFvalue(int idx,int idy)
{
	return data[idx * Y_SIZE + idy];
}

void AstarPathFinder::GetPath_ESDFvalue(int &sum_esdf,double &ave_esdf)
{
	getPath();
	sum_esdf = 0;
	ave_esdf = 0;

	if(globalpath.size() > 0)
	{
		int globalpath_num = globalpath.size();
		for (int i = 0; i < globalpath_num; i++)
		{
			sum_esdf += getESDFvalue(globalpath[i][0],globalpath[i][1]);
		}
		
		// 求解平均值
		ave_esdf = sum_esdf/(globalpath_num*1.0f);
	}
	else
	{
		return;
	}
}


// 用于设定相关参数，一定要放到代码初始化之前
// g数值的权重
// h数值的权重
// g_distance 的计算方法 0 1 2 3方法 欧氏距离 曼哈顿 切比学夫 对角线
// h_distance 的计算方法
void AstarPathFinder::setParams(double wight_g, double wight_h, int g_distance, int h_distance,double s_radius)
{
	g_method = g_distance; // 默认使用对角线距离
	h_method = h_distance; // 默认使用对角线距离

	g_value = wight_g; // 默认权重是1
	h_value = wight_h; // 默认权重是1

	search_radius = s_radius;

	// std::cout<<"G-method: "<<g_method<<std::endl;
	// std::cout<<"H-method: "<<h_method<<std::endl;

	// std::cout<<"G-weight: "<<g_value<<std::endl;
	// std::cout<<"H-weight: "<<h_value<<std::endl;
}

/**
 * @brief 栅格地图坐标系转世界坐标系
 * @param mx   地图坐标x
 * @param my   地图坐标y
 * @param wx   世界坐标x
 * @param wy   世界坐标y
 * @return
 * @attention
 * @todo
 * */
void AstarPathFinder::mapToWorld(int mx, int my, double &wx, double &wy) const
{
	wx = origin_x + (mx + 0.5) * resolution_;
	wy = origin_y + (my + 0.5) * resolution_;
}

/**
 * @brief 世界坐标系转栅格地图坐标系
 * @param wx   世界坐标x
 * @param wy   世界坐标y
 * @param mx   地图坐标x
 * @param my   地图坐标y
 * @return
 * @attention
 * @todo
 * */
bool AstarPathFinder::worldToMap(double wx, double wy, int &mx, int &my) const
{
	if (wx < origin_x || wy < origin_y)
		return false;

	mx = (int)(1.0 * (wx - origin_x) / resolution_);
	my = (int)(1.0 * (wy - origin_y) / resolution_);

	if (mx < X_SIZE && my < Y_SIZE)
		return true;

	return false;
}

/**
 * @brief Astar初始化(面向用户)
 * @param _resolution 地图分辨率
 * @param _originx   地图实际的起点
 * @param _originy   地图实际的终点
 * @param 地图x方向上的尺寸
 * @param 地图y方向上的尺寸
  nh.param("dwa/map_x_origin",map_x_origin,0);
 * @param 一维的地图数据
 * @return
 * @attention
 * @todo
 * */
void AstarPathFinder::InitMap(double _resolution, double _originx, double _originy,
															int _x_size, int _y_size, std::vector<int32_t> _mapData)
{
	resolution_ = _resolution;

	Y_SIZE = _y_size;
	X_SIZE = _x_size;

	origin_x = _originx;
	origin_y = _originy;

	// 初始化一个数组,按照XYZ的大小去初始化数组
	data = new int32_t[Y_SIZE * X_SIZE]; // 为将地图转化为8进制栅格地图作准备
	// 内存处理,清空数组
	memset(data, 0, Y_SIZE * X_SIZE * sizeof(int32_t));
	for (int i = 0; i < X_SIZE; i++)
	{
		for (int j = 0; j < Y_SIZE; j++)
		{
      // 设定障碍物
			data[i * Y_SIZE + j] = _mapData[i * Y_SIZE + j];
			if(data[i * Y_SIZE + j] >= OBSTACLE)
			{
				OBSTACLE = data[i * Y_SIZE + j];
			}
		}
		// cout<<endl;
	}

	// 分配栅格地图
	GridNodeMap = new GridNodePtr *[X_SIZE]; // 二级指针
	for (int i = 0; i < X_SIZE; i++)
	{
		GridNodeMap[i] = new GridNodePtr[Y_SIZE]; // x坐标系栅格化
		for (int j = 0; j < Y_SIZE; j++)
		{
			Vector2i tmpIdx(i, j);
			GridNodeMap[i][j] = new GridNode(tmpIdx); // 初始化栅格地图
		}
	}

	// 查看一维地图显示是否成功
	// std::cout << "地图长度x: " << X_SIZE << std::endl;
	// std::cout << "地图宽度y: " << Y_SIZE << std::endl;
}

// 获取系统时间 精确到ns
uint64_t AstarPathFinder::getSystemNSec(void)
{
	return ros::Time::now().toNSec();
}

// 搜索路径判断
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets, vector<double> &edgeCostSets, vector<double> &aclDist)
{
	// 邻近节点清空
	neighborPtrSets.clear();
	edgeCostSets.clear();
	aclDist.clear();

	// 开始完成A*算法实现
	// 以向量坐标形式呈现
	Eigen::Vector2i current_index = currentPtr->index;
	int current_x = current_index[0];
	int current_y = current_index[1];

	int n_x, n_y; // 定义搜索节点
	GridNodePtr tmp_ptr = NULL;
	Eigen::Vector2i tmp_index;

	for (int i = -1; i <= 1; ++i)
	{ // 以26连通形式进行探索
		for (int j = -1; j <= 1; ++j)
		{
			if (i == 0 && j == 0)
			{ // 若同时为0结束当前一轮探索，因为都是0的时候探索的点是当前点
				continue;
			}

			n_x = current_x + i;
			n_y = current_y + j;
			// 经过处理后，地图上所有要素的点均是大于0且小于地图最大尺寸的，
			// 若有点不在这个范围内，则结束当前一轮探索，开始下一轮
			if ((n_x < 0) || (n_y < 0) || (n_x > X_SIZE - 1) || (n_y > Y_SIZE - 1))
			{
				continue;
			}
			// 若点被占据则同上
			if (isOccupied(n_x, n_y))
			{
				continue;
			}

			tmp_ptr = GridNodeMap[n_x][n_y];
			double dist = getGhi(currentPtr, tmp_ptr);
			double distance;

			distance = sqrt(
					(currentPtr->index(0) - tmp_ptr->index(0)) * (currentPtr->index(0) - tmp_ptr->index(0)) +
					(currentPtr->index(1) - tmp_ptr->index(1)) * (currentPtr->index(1) - tmp_ptr->index(1)));

			neighborPtrSets.push_back(tmp_ptr);
			edgeCostSets.push_back(dist);
			aclDist.push_back(distance);
		}
	}
}

// 判断点是否被占据的具体实现形式
inline bool AstarPathFinder::isOccupied(const int &idx, const int &idy) const
{
	return (idx < X_SIZE && idy < Y_SIZE && (data[idx * Y_SIZE + idy] == OBSTACLE));
}



double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	// 选择你想要的启发式函数，曼哈顿距离(走格子)、欧式距离(两点之间线段最短)、对角线(18连通区域)或者0
	// 记得tie_breaker 平衡打断，尤其是在使用曼哈顿距离的时候会出现相同路径
	int distance_norm = h_method;
	double h = 0;
	Eigen::Vector2i start_index = node1->index;
	Eigen::Vector2i end_index = node2->index;
	switch (distance_norm)
	{
	case Euclidean:
	{
		double dx = abs((double)(start_index(0) - end_index(0)));
		double dy = abs((double)(start_index(1) - end_index(1)));
		h = std::sqrt((pow(dx, 2.0) + pow(dy, 2.0)));
		break;
	}
	case Manhattan:
	{
		double dx = abs((double)(start_index(0) - end_index(0)));
		double dy = abs((double)(start_index(1) - end_index(1)));
		h = dx + dy;
		break;
	}
	case L_infty:
	{
		double dx = abs((double)(start_index(0) - end_index(0)));
		double dy = abs((double)(start_index(1) - end_index(1)));
		h = max({dx, dy});
		break;
	}
	case Diagonal:
	{
		double distance[2];
		distance[0] = abs((double)(start_index(0) - end_index(0)));
		distance[1] = abs((double)(start_index(1) - end_index(1)));
		sort(distance, distance + 2);
		h = distance[0] + distance[1] + (std::sqrt(2.0) - 2) * distance[0];
		break;
	}

	default:
		break;
	}

	// 若出现距离相同的路径打破平衡
	if (use_Tie_breaker)
	{
		double p = 1 / 25;
		h = h * (1.0 + p);
	}
	double obs_num = 0.0;
	if(is_use_esdf == true)
	{
		obs_num = (double)(data[start_index(0)*Y_SIZE + start_index(1)]);
	}
	else{
		obs_num = 0;
	}

	// cout<<"obs_num:"<<obs_num<<endl;
	return h + obs_num; // 返回计算出来的规划轨迹距离
}

double AstarPathFinder::getGhi(GridNodePtr node1, GridNodePtr node2)
{
	// 选择你想要的启发式函数，曼哈顿距离(走格子)、欧式距离(两点之间线段最短)、对角线(18连通区域)或者0

	// 记得tie_breaker 平衡打断，尤其是在使用曼哈顿距离的时候会出现相同路径
	int distance_norm = g_method;
	double g = 0;
	Eigen::Vector2i start_index = node1->index;
	Eigen::Vector2i end_index = node2->index;

	switch (distance_norm)
	{
	case Euclidean:
	{
		double dx = abs((double)(start_index(0) - end_index(0)));
		double dy = abs((double)(start_index(1) - end_index(1)));
		g = std::sqrt((pow(dx, 2.0) + pow(dy, 2.0)));
		break;
	}
	case Manhattan:
	{
		double dx = abs((double)(start_index(0) - end_index(0)));
		double dy = abs((double)(start_index(1) - end_index(1)));
		g = dx + dy;
		break;
	}
	case L_infty:
	{
		double dx = abs((double)(start_index(0) - end_index(0)));
		double dy = abs((double)(start_index(1) - end_index(1)));
		g = max({dx, dy});
		break;
	}
	case Diagonal:
	{
		double distance[2];
		distance[0] = abs((double)(start_index(0) - end_index(0)));
		distance[1] = abs((double)(start_index(1) - end_index(1)));
		sort(distance, distance + 2);
		g = distance[0] + distance[1] + (std::sqrt(2.0) - 2) * distance[0];
		break;
	}

	default:
		break;
	}

	return g; // 返回计算出来的规划轨迹距离
}

// A*路径搜索，直接放入世界坐标系下的坐标即可进行搜索
int AstarPathFinder::AstarWorldSearch(Vector2d startpoint, Vector2d endpoint)
{
	Vector2i node1, node2;
	worldToMap(startpoint[0], startpoint[1], node1[0], node1[1]);
	worldToMap(endpoint[0], endpoint[1], node2[0], node2[1]);

	int flag =  AstarGraphSearch(node1, node2);
	return flag;
}

// 检测当前点的合法性，主要是是否在障碍物中
bool AstarPathFinder::StartOrEndValidCheck(Vector2d startpoint, Vector2d endpoint)
{
	Vector2i node1=Vector2i(0,0), node2=Vector2i(0,0);

	worldToMap(startpoint[0], startpoint[1], node1[0], node1[1]);
	worldToMap(endpoint[0], endpoint[1], node2[0], node2[1]);

	if (isOccupied(node1[0], node1[1]) || isOccupied(node2[0], node2[1]))
	{
		std::cout << "Failed to get a path. point in obstacle." << std::endl;
		return false;
	}

	// 当到达终点
	if(node1[0] == node2[0] && node1[1] == node2[1])
	{
		std::cout << "Arrived the end point." << std::endl;
		return false;
	}

	else
	{
		return true;
	}
}

// A*路径搜索
int AstarPathFinder::AstarGraphSearch(Vector2i start_pt, Vector2i end_pt)
{
	// 记录路径搜索需要的时间
	uint64_t time_1 = getSystemNSec();

  if (isOccupied(start_pt[0], start_pt[1]))
  {
    std::cout << "start point is" << start_pt[0] << "  " << end_pt[1] << std::endl;
    std::cout << "failed to get a path.start point is obstacle." << std::endl;
    return IN_OCCUPIED;
  }
  if (isOccupied(end_pt[0], end_pt[1]))
  {
    std::cout << "target point is" << end_pt[0] << "  " << end_pt[1] << std::endl;
    std::cout << "failed to get a path.target point is obstacle." << std::endl;
    return IN_OCCUPIED;
  }

	// 初始化起点和终点节点(因为前面已经初始化过，所以不需要再New)
	GridNodePtr startPtr = GridNodeMap[start_pt(0)][start_pt(1)];
	GridNodePtr endPtr = GridNodeMap[end_pt(0)][end_pt(1)];

	startpoint = start_pt;
	endpoint = end_pt;

	goalIdx = end_pt;

	// 待弹出点集
	openSet.clear();

	// 定义要弹出的节点
	GridNodePtr currentPtr = NULL;
	GridNodePtr neighborPtr = NULL;

	// 计算启发函数
	startPtr->gScore = 0;
	startPtr->fScore = getHeu(startPtr, endPtr);
	startPtr->aclDistance = 0; // 初始化已走过实际距离为0

	// 将起点加入开集
	// 自由点为0 闭集为-1 开集为1
	startPtr->id = 1;
	startPtr->nodeMapIt = openSet.insert(make_pair(startPtr->fScore, startPtr));

	// 预先定义拓展队列和cost队列
	vector<GridNodePtr> neighborPtrSets;
	vector<double> edgeCostSets;
	vector<double> aclDist;

	// 定义搜索终点
	terminatePtr = NULL;
	// 定义缓冲范围
	const int tolerance = ceil(1/resolution_);

	// this is the main loop
	while (!openSet.empty())
	{
		// 弹出最大f的节点
		currentPtr = openSet.begin()->second;
		currentPtr->id = -1; // 标记为闭集

		double tempWorldx,tempWorldy;
		mapToWorld(currentPtr->index(0),currentPtr->index(1),tempWorldx,tempWorldy);
		double startWorldx,startWorldy;
		mapToWorld(startPtr->index(0),startPtr->index(1),startWorldx,startWorldy);


		// 判断是否在规划半径区域内 一个圆环范围
		bool reach_horizon = sqrt(
															pow(abs(tempWorldx-startWorldx),2)+
															pow(abs(tempWorldy-startWorldy),2)
															) >= search_radius;

		// 接近终点
		bool near_end = abs(currentPtr->index(0) - endPtr->index(0) <= tolerance) &&
											abs(currentPtr->index(1) - endPtr->index(1)) <= tolerance;
		
		bool is_shot_succ_ = false;
		// 达到搜索范围或者接近终点
		if(reach_horizon || near_end)
		{
			// 终止节点等于当前节点
			terminatePtr = currentPtr;
			// 接近终点
			if(near_end)
			{
				// cout<<"near end!!!"<<endl;
			}
		}

    // 如果到达范围
    if (reach_horizon)
    {
			// 到达同一平面
			std::cout << "reach horizon" << std::endl;
			return REACH_HORIZON;
    }

		// 从开集中移除
		openSet.erase(openSet.begin());
		// 获取拓展集合
		AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets, aclDist);
		// 遍历拓展集合
		for (int i = 0; i < (int)neighborPtrSets.size(); i++)
		{

			neighborPtr = neighborPtrSets[i];
			double gh = g_value * (currentPtr->gScore + edgeCostSets[i]); // 计算走过距离代价数值
			/* 计算实际走过的距离 */
			double aclDistance = currentPtr->aclDistance + aclDist[i];
			double fh = gh + h_value * getHeu(neighborPtr, endPtr); // 总的代价数值计算
			// 如果为自由节点
			if (neighborPtr->id == 0)
			{
				// 计算相应的g和f，并加入opensets
				neighborPtr->gScore = gh;
				neighborPtr->fScore = fh;
				neighborPtr->aclDistance = aclDistance; // 传递参数
				neighborPtr->cameFrom = currentPtr;
				neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr)); // 此处注意一定要先计算和赋值完f再加入

				// 判断是否为目标节点 改到此处为了提高代码效率 不用将所有节点加入后等弹出时发现目标再退出
				if (neighborPtr->index == goalIdx)
				{
					uint64_t time_2 = getSystemNSec(); // 获取当前系统时间
					terminatePtr = neighborPtr;
					ParamsLast.time = (double)(time_2 - time_1) / 1000000.0f;
					// cout<<"ParamsLast.time:"<<ParamsLast.time<<endl;
					ParamsLast.dist = currentPtr->aclDistance;
					findFlag = true;
					return REACH_END;
				}
				else
				{
					// 标记为open list
					neighborPtr->id = 1;
					continue;
				}
			}
			// id为1的话代表其为障碍物
			else if (neighborPtr->id == 1)
			{
				// 如果邻居节点的g参数大于gh值,进行处理
				if (neighborPtr->gScore > gh)
				{
					// 更新对应的f值
					neighborPtr->gScore = gh;
					neighborPtr->fScore = fh;
					// 传参用于计算实际走过的距离
					neighborPtr->aclDistance = aclDistance;
					neighborPtr->cameFrom = currentPtr;
					openSet.erase(neighborPtr->nodeMapIt); // 擦除这个节点
					neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
				}
			}
			else
			{
				// 如果是closelist里面的则不做处理
				continue;
			}
		}
	}
	// 如果搜索失败
	if (ParamsLast.time > 1000000)
	{
		std::cout << "Time consume in Astar path finding is " << ParamsLast.time << std::endl;
		std::cout << "Path find failed." << std::endl;
		findFlag = false;
		return NO_PATH;
	}
}

Vector2d AstarPathFinder::getTerminate(void)
{
	double worldx,worldy;

	mapToWorld(terminatePtr->index(0),terminatePtr->index(1),worldx,worldy);
	
	Vector2d tempGoal = Vector2d(worldx,worldy);

	return tempGoal;
}

// A*复位栅格参数
void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
	ptr->id = 0;
	ptr->cameFrom = NULL;
	ptr->gScore = inf;
	ptr->fScore = inf;
}

// A*复位已用栅格参数
void AstarPathFinder::resetUsedGrids()
{
	for (int i = 0; i < X_SIZE; i++)
		for (int j = 0; j < Y_SIZE; j++)
			resetGrid(GridNodeMap[i][j]);
}

// 获得访问结点
vector<Vector2i> AstarPathFinder::getVisitedNodes()
{
	vector<Vector2i> visited_nodes;
	for (int i = 0; i < X_SIZE; i++)
		for (int j = 0; j < Y_SIZE; j++)
		{
			if (GridNodeMap[i][j]->id == -1) // 仅显示关闭列表中的节点
			{
				visited_nodes.push_back(GridNodeMap[i][j]->index);
			}
		}

	// 最终访问节点数目
	return visited_nodes;
}

int AstarPathFinder::getVisitedNodesNum(void)
{
	vector<Vector2i> visited_nodes = getVisitedNodes();
	// 返回访问的节点总数
	return visited_nodes.size();
}

// 获得访问结点直接输出到世界坐标系
vector<Vector2d> AstarPathFinder::getVisitedWorldNodes()
{
	vector<Vector2d> visited_nodes;
	for (int i = 0; i < X_SIZE; i++)
		for (int j = 0; j < Y_SIZE; j++)
		{
			if (GridNodeMap[i][j]->id == -1) // 仅显示关闭列表中的节点
			{
				Vector2d worldpoint;
				mapToWorld(GridNodeMap[i][j]->index[0], GridNodeMap[i][j]->index[1], worldpoint[0], worldpoint[1]);
				visited_nodes.push_back(worldpoint);
			}
		}

	// 最终访问节点数目
	return visited_nodes;
}

// 获取规划好的路径
void AstarPathFinder::getPath(void)
{
	// vector<Vector2i> path;
	globalpath.clear();
	vector<GridNodePtr> gridPath;

	GridNodePtr tmp_ptr = terminatePtr;
	while (tmp_ptr->cameFrom != NULL)
	{
		gridPath.push_back(tmp_ptr);
		tmp_ptr = tmp_ptr->cameFrom;
	}

	for (auto ptr : gridPath)
		globalpath.push_back(ptr->index);

	globalpath.push_back(startpoint);

	reverse(globalpath.begin(), globalpath.end());

	double firstDis;
	Vector2i fistNode = globalpath[0];
	firstDis = sqrt(
			(double)((fistNode(0) - startpoint(0)) * (fistNode(0) - startpoint(0))) +
			(double)((fistNode(1) - startpoint(1)) * (fistNode(1) - startpoint(1))));

	world_distance = firstDis + ParamsLast.dist;
}

// 提取路径采样点
vector<Vector2d> AstarPathFinder::getSamples(double distance)
{
	allpath.clear();
	// 获取全局路径
	allpath = getWorldPath();

	int allpath_num = allpath.size();

	double path_length = getWorldPathLength();

	// 求解分段数目 分多少段
	int k_num = int(path_length/distance);
	// 计算每段数目
	int num = allpath_num/k_num;

	// 定义新的路径
	vector<Vector2d> newpath;
	for (int i = 0; i < k_num; i++)
	{
		newpath.push_back(allpath[i*num]);
	}
	newpath.push_back(allpath[allpath_num-1]);

	return newpath;
}

// 获取世界路径，最终在世界坐标系下的路径
vector<Vector2d> AstarPathFinder::getWorldPath(void)
{
	vector<Vector2d> path;
	vector<GridNodePtr> gridPath;

	GridNodePtr tmp_ptr = terminatePtr;
	while (tmp_ptr->cameFrom != NULL)
	{
		gridPath.push_back(tmp_ptr);
		tmp_ptr = tmp_ptr->cameFrom;
	}

	for (auto ptr : gridPath)
	{
		Vector2d worldpoint;
		mapToWorld(ptr->index[0], ptr->index[1], worldpoint[0], worldpoint[1]);
		path.push_back(worldpoint);
	}

	Vector2d worldpoint;
	mapToWorld(startpoint[0], startpoint[1], worldpoint[0], worldpoint[1]);
	path.push_back(worldpoint);

	reverse(path.begin(), path.end());

	// 最终路径走过的节点数目
	// std::cout<<"pathsize:"<<path.size()<<std::endl;
	// std::cout<<"startpoint: "<<path[0][0]<<" "<<path[0][obstacle1]<<std::endl;

	double firstDis;
	Vector2i firstNode = startpoint;
	worldToMap(path[0][0], path[1][1], firstNode[0], firstNode[1]);
	firstDis = sqrt(
			(double)((firstNode[0] - startpoint[0]) * (firstNode[0] - startpoint[0])) +
			(double)((firstNode[1] - startpoint[1]) * (firstNode[1] - startpoint[1])));

	world_distance = firstDis + ParamsLast.dist;

	// if(findFlag == true)
	//     std::cout<<"Path cost is"<<ParamsLast.dist+firstDis<<"length-unit"<<std::endl;
	// else
	//     std::cout<<"Path find failed"<<std::endl;

	return path;
}
//获取总的路径点的长度
double AstarPathFinder::getWorldPathLength(void)
{
	// 计算总的路径长度
	double all_length = 0.0;
	vector<Vector2d> worldpath = getWorldPath();
	for (int i = 1; i < worldpath.size(); i++)
	{
		all_length += calPointLength(worldpath[i],worldpath[i-1]);
	}

	return all_length;
}

bool AstarPathFinder::AstarSearchResult(void)
{
	return findFlag;
}
