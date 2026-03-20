/***
 * @          佛曰:
 * @                  写字楼里写字间，写字间里程序员；
 * @                  程序人员写程序，又拿程序换酒钱。
 * @                  酒醒只在网上坐，酒醉还来网下眠；
 * @                  酒醉酒醒日复日，网上网下年复年。
 * @                  但愿老死电脑间，不愿鞠躬老板前；
 * @                  奔驰宝马贵者趣，公交自行程序员。
 * @                  别人笑我忒疯癫，我笑自己命太贱；
 * @                  不见满街漂亮妹，哪个归得程序员？
 * @
 * @Author: your name
 * @Date: 2022-12-03 20:30:41
 * @LastEditTime: 2023-01-06 20:00:18
 * @LastEditors: your name
 * @Description:
 * @FilePath: /tianbot_mini/src/astar_super/src/path_searcher/JPS_searcher.cpp
 * @可以输入预定的版权声明、个性签名、空行等
 */

#include "path_searcher/JPS_searcher.h"

using namespace std;
using namespace Eigen;

JPSPathFinder::JPSPathFinder()
{
  jn2d = new JPS2DNeib(); // 创建

  // std::cout<<"JPS Config"<<std::endl;

  g_method = Manhattan; // 默认使用曼哈顿距离
  h_method = Manhattan; // 默认使用曼哈顿距离

  g_value = 1.0; // 默认权重是1
  h_value = 1.0; // 默认权重是1

  // std::cout<<"G-method: "<<g_method<<std::endl;
  // std::cout<<"H-method: "<<h_method<<std::endl;

  // std::cout<<"G-weight: "<<g_value<<std::endl;
  // std::cout<<"H-weight: "<<h_value<<std::endl;
}

// 用于传参
void JPSPathFinder::InitParams(ros::NodeHandle &nh)
{
	nh.param("jps_weight/g", g_value, 1.0);          // 权重a值
	nh.param("jps_weight/h", h_value, 1.0);          // 权重b值
	nh.param("jps_heuristic/distance", h_method,0); // 0 1 2 3方法 欧氏距离 曼哈顿 切比学夫 对角线
	nh.param("jps_glength/distance",g_method ,0);

	nh.param("jps_search/radius", search_radius, 100.0);	
	nh.param("jps_search/is_use_esdf", is_use_esdf, false);	
}

// 判断跳点是否被占据具体实现
inline bool JPSPathFinder::isOccupied(const int &idx_x, const int &idx_y) const
{
  return (idx_x < X_SIZE && idx_y < Y_SIZE && (data[idx_x * Y_SIZE + idx_y] == OBSTACLE));
}

// 判断跳点是否是自由的具体实现形式
inline bool JPSPathFinder::isFree(const int &idx_x, const int &idx_y) const
{
  return (idx_x < X_SIZE && idx_y < Y_SIZE && (data[idx_x * Y_SIZE + idx_y] < OBSTACLE));
}

// 判断跳点是否是自由的
inline bool JPSPathFinder::isFree(const Eigen::Vector2i &index) const
{
  return isFree(index(0), index(1));
}

// 搜索路径判断，原理有点与最优化中的步长加速法相似
inline void JPSPathFinder::JPSGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets, vector<double> &edgeCostSets, vector<double> &aclDist)
{
  // 邻近节点清空
  neighborPtrSets.clear();
  edgeCostSets.clear();
  aclDist.clear();
  // 计算向量模长，其绝对值相加
  const int norm1 = abs(currentPtr->dir(0)) + abs(currentPtr->dir(1));

  int num_neib = jn2d->nsz[norm1][0];  // 邻近节点数
  int num_fneib = jn2d->nsz[norm1][1]; // 强制节点数

  int id = (currentPtr->dir(0) + 1) + 3 * (currentPtr->dir(1) + 1);

  for (int dev = 0; dev < num_neib + num_fneib; ++dev)
  {
    Vector2i neighborIdx; // 邻居节点
    Vector2i expandDir;   // 扩展节点坐标向量形式
    /****************************************************************/
    // 将节点进行分割归类，普通节点放到前面，强制邻居接节点放到后面
    // 这样便于之后的数据处理
    /* 当dev小于普通邻居的扩展书目的时候 */
    if (dev < num_neib)
    { // 扩展节点坐标，这里可以看作是普通节点
      expandDir(0) = jn2d->ns[id][0][dev];
      expandDir(1) = jn2d->ns[id][1][dev];

      // 跳点检测
      if (!jump(currentPtr->index, expandDir, neighborIdx))
        continue;
    }
    else
    {
      int nx = currentPtr->index(0) + jn2d->f1[id][0][dev - num_neib];
      int ny = currentPtr->index(1) + jn2d->f1[id][1][dev - num_neib];

      // 判断当前节点是否被占据
      if (isOccupied(nx, ny))
      {
        expandDir(0) = jn2d->f2[id][0][dev - num_neib];
        expandDir(1) = jn2d->f2[id][1][dev - num_neib];

        // 如果跳点失败，那么结束当前循环，进行下一轮循环
        if (!jump(currentPtr->index, expandDir, neighborIdx))
          continue;
      }
      else
        continue;
    }

    /****************************************************************/
    // 最终节点以坐标的形式体现
    GridNodePtr nodePtr = GridNodeMap[neighborIdx(0)][neighborIdx(1)];
    nodePtr->dir = expandDir;

    // 跳点搜索结束开始设定节点
    neighborPtrSets.push_back(nodePtr);
    double dist; // 用于计算历史代价数值
    switch (g_method)
    {
    case Euclidean:
    {
      double dx = abs((double)(neighborIdx(0) - currentPtr->index(0)));
      double dy = abs((double)(neighborIdx(1) - currentPtr->index(1)));
      dist = std::sqrt((std::pow(dx, 2.0) + std::pow(dy, 2.0)));
      break;
    }
    case Manhattan:
    {
      double dx = abs((double)(neighborIdx(0) - currentPtr->index(0)));
      double dy = abs((double)(neighborIdx(1) - currentPtr->index(1)));
      dist = dx + dy;
      break;
    }
    case L_infty:
    {
      double dx = abs((double)(neighborIdx(0) - currentPtr->index(0)));
      double dy = abs((double)(neighborIdx(1) - currentPtr->index(1)));
      dist = std::max({dx, dy});
      break;
    }
    case Diagonal:
    {
      double Distance[2];
      Distance[0] = abs((double)(neighborIdx(0) - currentPtr->index(0)));
      Distance[1] = abs((double)(neighborIdx(1) - currentPtr->index(1)));
      std::sort(Distance, Distance + 2);
      dist = Distance[0] + Distance[1] + (std::sqrt(2.0) - 2) * Distance[0];
      break;
    }

    default:
      break;
    }
    dist = dist * g_value; // 距离*权重
    // 用欧氏距离计算g数值
    edgeCostSets.push_back(dist);

    double distance = sqrt(
        (neighborIdx(0) - currentPtr->index(0)) * (neighborIdx(0) - currentPtr->index(0)) +
        (neighborIdx(1) - currentPtr->index(1)) * (neighborIdx(1) - currentPtr->index(1)));
    aclDist.push_back(distance);
  }
}

// 跳点路径搜索判断点是否可用
bool JPSPathFinder::jump(const Vector2i &curIdx, const Vector2i &expDir, Vector2i &neiIdx)
{
  // ROS_WARN("start jump!");
  // 邻居节点坐标等于当前节点+扩展向量
  neiIdx = curIdx + expDir;

  if (!isFree(neiIdx)) // 此点非空无法跳点
    return false;

  if (neiIdx == goalIdx) // 如果是目标点可以跳
    return true;

  if (hasForced(neiIdx, expDir)) // 如果是强制节点可以跳
    return true;

  const int id = (expDir(0) + 1) + 3 * (expDir(1) + 1);
  // 这里计算扩展方向距离父节点的距离
  const int norm1 = abs(expDir(0)) + abs(expDir(1));
  int num_neib = jn2d->nsz[norm1][0]; // 根据距离判断节点的数目

  // 查询周围的点，如果满足条件可以跳点
  for (int k = 0; k < num_neib - 1; ++k)
  {
    Vector2i newNeiIdx;
    // 新的方向继续进行探索
    Vector2i newDir(jn2d->ns[id][0][k], jn2d->ns[id][1][k]);
    if (jump(neiIdx, newDir, newNeiIdx)) // 不断进行迭代
      return true;
  }

  // 迭代筛选路径
  return jump(neiIdx, expDir, neiIdx);
}

// 判断是否已经强制
inline bool JPSPathFinder::hasForced(const Vector2i &idx, const Vector2i &dir)
{
  // 扩展方向距离的计算
  int norm1 = abs(dir(0)) + abs(dir(1));
  // 每个格子按照次序进行排列
  int id = (dir(0) + 1) + 3 * (dir(1) + 1);

  // 这里的邻居节点判断方式就是以单位距离的平移和srqt(2)的对角线平移
  switch (norm1)
  {
  case 1:
    //  距离为1的时候,判断8个强迫邻居,就是各个平面的中心点,
    // 相邻的邻居就是8个
    for (int fn = 0; fn < 2; ++fn)
    {
      int nx = idx(0) + jn2d->f1[id][0][fn];
      int ny = idx(1) + jn2d->f1[id][1][fn];
      if (isOccupied(nx, ny)) // 判断是否已被占据
        return true;
    }
    return false;

  case 2:
    //  当距离为2的时候,判断8个强迫邻居
    for (int fn = 0; fn < 2; ++fn)
    {
      int nx = idx(0) + jn2d->f1[id][0][fn];
      int ny = idx(1) + jn2d->f1[id][1][fn];
      if (isOccupied(nx, ny)) // 判断是否已被占据
        return true;
    }
    return false;
  default:
    return false;
  }
}

// JPS路径搜索，直接放入世界坐标系下的坐标即可进行搜索
int JPSPathFinder::JPSWorldSearch(Vector2d startpoint, Vector2d endpoint)
{
  Vector2i node1, node2;
  worldToMap(startpoint[0], startpoint[1], node1[0], node1[1]);
  worldToMap(endpoint[0], endpoint[1], node2[0], node2[1]);

  return JPSGraphSearch(node1, node2);
}

int JPSPathFinder::JPSGraphSearch(Eigen::Vector2i start_pt, Eigen::Vector2i end_pt)
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

  // 将起点放置在open-list 中
  startPtr->gScore = 0;
  startPtr->fScore = h_value * getHeu(startPtr, endPtr);
  // 起点处已走过距离设定为0
  startPtr->aclDistance = 0;

  // 第一步完成获取 f参数，以上操作是启发式探索函数
  startPtr->id = 1;
  startPtr->nodeMapIt = openSet.insert(make_pair(startPtr->fScore, startPtr));

  // 第二步，在主循环开始之前做必要的准备工作
  double tentative_gScore;
  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;
  vector<double> aclDist;

	// 定义缓冲范围
	const int tolerance = ceil(1/resolution_);

  // 主循环
  while (!openSet.empty())
  {
    // 第三步：把open-list中的最低节点移动到closed-list中
    // 注意：这部分需要使用C++ STL库，multimap检索路径等库
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

    openSet.erase(openSet.begin()); // 从open-list中擦除节点

    // 获取成功
    JPSGetSucc(currentPtr, neighborPtrSets, edgeCostSets, aclDist); // we have done it for you

    // 第四步：检索 所有无法扩展的邻居节点，结束这次循环
    for (int i = 0; i < (int)neighborPtrSets.size(); i++)
    {
      // 判断已经探索的邻居节点是否有效
      neighborPtr = neighborPtrSets[i];

      // 趋势g值,因为JPS的跳点搜索方式不是一步一步向外拓展,而是
      // 直线延伸,所以需要判断这个拓展的趋势的g值状态
      tentative_gScore = currentPtr->gScore + edgeCostSets[i];
      double tentative_aclDist = currentPtr->aclDistance + aclDist[i]; // 计算趋势距离

      if (neighborPtr->id == 0)
      { // 发现一个新的节点
        // 对于一个新的节点，判断是否所需要，并将其相邻节点放置到open-list中并记录
        neighborPtr->gScore = tentative_gScore;                                              // 重新设定g参数
        neighborPtr->fScore = neighborPtr->gScore + h_value * getHeu(neighborPtr, endPtr); // 重新设定f参数
        neighborPtr->aclDistance = tentative_aclDist;                                        // 计算已经走过的距离
        neighborPtr->cameFrom = currentPtr;

        // x y向量计算
        for (int i = 0; i < 2; i++)
        {
          neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
          if (neighborPtr->dir(i) != 0)
            neighborPtr->dir(i) /= abs(neighborPtr->dir(i));
        }

        // 邻居节点对应到地图上的点
        neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));

        // 如果邻居节点是设定的目标节点,那么搜索结束算法完成
        if (neighborPtr->index == goalIdx)
        {
          uint64_t time_2 = getSystemNSec();
          terminatePtr = neighborPtr;
          JPSLastParams.lastTime = (double)(time_2 - time_1) / 10000000;
          JPSLastParams.lastDist = currentPtr->aclDistance;
          findFlag = true;
          return REACH_END;
        }
        else
        {
          neighborPtr->id = 1;
          continue;
        }
      }
      // 如果邻居节点的id是1,并且趋势g值小于邻居节点的g值
      else if (neighborPtr->id == 1 && tentative_gScore <= neighborPtr->gScore)
      { // 在open-list中更新新的节点
        // 在open-list中更新节点，保持在openset中的状态，之后将邻近节点放到open-list中并记录
        neighborPtr->gScore = tentative_gScore;
        /* 计算实际走过距离 */
        neighborPtr->aclDistance = tentative_aclDist;

        // 启发式算法进行搜索
        neighborPtr->fScore = neighborPtr->gScore + h_value * getHeu(neighborPtr, endPtr);
        neighborPtr->cameFrom = currentPtr;

        // x y z 三个坐标的赋值搜索
        for (int i = 0; i < 2; i++)
        {
          neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i); // 搜索邻近节点
          if (neighborPtr->dir(i) != 0)                                       // 判断方式和以上相同
            neighborPtr->dir(i) /= abs(neighborPtr->dir(i));                  // 求取单位向量,得出的就是方向趋势
        }

        // 节点擦除
        openSet.erase(neighborPtr->nodeMapIt);
        neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
      }
      else
      {
        continue;
      }
    }
  }
  // 如果搜索失败
  uint64_t time_2 = getSystemNSec();
  if (time_2 - time_1 > 100000000)
  {
    std::cout << "Time consume in Astar path finding is " << (time_2 - time_1) << std::endl;
    std::cout << "Path find failed." << std::endl;
    findFlag = false;
  }
}

vector<Vector2i> JPSPathFinder::getJPSPath()
{
  vector<Vector2i> path;
  vector<GridNodePtr> gridPath;

  GridNodePtr tmp_ptr = terminatePtr;
  while (tmp_ptr->cameFrom != NULL)
  {
    gridPath.push_back(tmp_ptr);
    tmp_ptr = tmp_ptr->cameFrom;
  }

  for (auto ptr : gridPath)
    path.push_back(ptr->index);

  path.push_back(startpoint); // 把第一个点放近来

  reverse(path.begin(), path.end());

  // 最终路径走过的节点数目
  // std::cout<<"pathsize:"<<path.size()<<std::endl;

  double firstDis;
  Vector2i fistNode = path[1];
  firstDis = sqrt(
      (double)((fistNode(0) - startpoint(0)) * (fistNode(0) - startpoint(0))) +
      (double)((fistNode(1) - startpoint(1)) * (fistNode(1) - startpoint(1))));

  world_distance = firstDis + JPSLastParams.dist;
  // if(findFlag == true)
  //     std::cout<<"Path cost is"<<JPSLastParams.dist+firstDis<<"length-unit"<<std::endl;
  // else
  //     std::cout<<"Path find failed"<<std::endl;

  return path;
}

// 获取世界路径，最终在世界坐标系下的路径
vector<Vector2d> JPSPathFinder::getJPSWorldPath(void)
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

  double firstDis;
  Vector2i firstNode = startpoint;
  worldToMap(path[0][0], path[1][1], firstNode[0], firstNode[1]);
  firstDis = sqrt(
      (double)((firstNode[0] - startpoint[0]) * (firstNode[0] - startpoint[0])) +
      (double)((firstNode[1] - startpoint[1]) * (firstNode[1] - startpoint[1])));

  world_distance = firstDis + JPSLastParams.dist;
  // if(findFlag == true)
  //     std::cout<<"Path cost is"<<JPSLastParams.dist+firstDis<<"length-unit"<<std::endl;
  // else
  //     std::cout<<"Path find failed"<<std::endl;

  return path;
}
