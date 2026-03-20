/*
 * @Author: your name
 * @Date: 2023-05-11 09:48:44
 * @LastEditTime: 2024-01-24 14:23:57
 * @LastEditors: your name
 * @Description:
 * @FilePath: /MyFormProject/src/robot_navigation/include/path_searcher/Hybrid_astar.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __HYBRID_ASTAR_H
#define __HYBRID_ASTAR_H

#include "iostream"
#include "cmath"
#include "ros/ros.h"
#include "ros/console.h"
#include <boost/functional/hash.hpp>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include "Eigen/Eigen"

#define INF 1 >> 22 // 1/(2^20)
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define OBSTACLE 100 // 障碍物代表数值100

using namespace std;
using namespace Eigen;

struct HybridGridNode;
typedef HybridGridNode *HybridGridNodePtr;

struct HybridGridNode
{
  int id;              // 1-->open set,-1-->closed set
  Eigen::Vector2i dir; // direction of enpanding
  Eigen::Vector2i index;

  double gScore, fScore; // 分类模型评估方法f分数的两个参数
  double aclDistance;    // 定义一个用于计算A*实际走过的距离的参数

  // 开辟状态矩阵用来存储运动状态，前两位位置状态即当前车的位置，后两位是车的速度 x y上的线速度
  Eigen::Matrix<double, 4, 1> state;

  // 输入量  先放这 什么用一会再说
  Eigen::Vector2d input;
  // 时间延时
  double duration;
  // 定义节点状态
  char node_state;

  // 当前节点的时间参数
  double time;  // dyn
  // 时间取整
  int time_idx;
  // 定义父节点
  HybridGridNodePtr cameFrom; // 就是fast planner 中的parent

  std::multimap<double, HybridGridNodePtr>::iterator nodeMapIt;

  HybridGridNode(Eigen::Vector2i _index)
  {
    id = 0;
    index = _index;
    dir = Eigen::Vector2i::Zero();

    gScore = INF; // 相当于无穷小的一个数,可看作归0
    fScore = INF; // 同上
    aclDistance = INF;
    cameFrom = NULL;
  }

  HybridGridNode(){};
  ~HybridGridNode(){};
};

class NodeComparator {
 public:
  bool operator()(HybridGridNodePtr node1, HybridGridNodePtr node2) {
    return node1->fScore > node2->fScore;
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (long int i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector2i, HybridGridNodePtr, matrix_hash<Eigen::Vector2i>>
      data_2d_;
  std::unordered_map<Eigen::Vector3i, HybridGridNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector2i idx, HybridGridNodePtr node) {
    data_2d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector2i idx, int time_idx, HybridGridNodePtr node) {
    data_3d_.insert(std::make_pair(
        Eigen::Vector3i(idx(0), idx(1) , time_idx), node));
  }

  // 
  HybridGridNodePtr find(Eigen::Vector2i idx) {
    auto iter = data_2d_.find(idx);
    return iter == data_2d_.end() ? NULL : iter->second;
  }
  // 
  HybridGridNodePtr find(Eigen::Vector2i idx, int time_idx) {
    auto iter =
        data_3d_.find(Eigen::Vector3i(idx(0), idx(1), time_idx));
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_2d_.clear();
    data_3d_.clear();
  }
};

// Astar路径规划类
class Hybrid_astar
{
private:
  // 搜索到的路径点
  std::vector<HybridGridNodePtr> path_nodes_;
  // 开辟路径点池或者说集合
  std::vector<HybridGridNodePtr> path_node_pool_;

  NodeHashTable expanded_nodes_;
  std::priority_queue<HybridGridNodePtr, std::vector<HybridGridNodePtr>, NodeComparator>
      open_set_;
  // 定义已使用节点数目  后面的未知知道了再写
  int use_node_num_, iter_num_;
  /* search */
  // 规划器允许的最大时间长度，即规划出的路径时间不能超过max_tau_
  // 控制最大的时间跨度，用于平衡搜索速度和搜索质量。值越大，搜索范围就越大，但搜索速度就会变慢
  double max_tau_;
  // 路径规划初始时的最大时间跨度，一般情况下会比 max_tau 大，用于加快搜索速度
  double init_max_tau_;
	// 最大速度 最大加速度
  double max_vel_, max_acc_;
	// 控制路径规划中时间成本的权重，用于平衡时间成本和距离成本。值越大，时间成本就越重要
  double w_time_;
  // 控制路径规划的搜索深度，即在未来多少秒内进行路径规划。值越大，搜索深度就越深
  double horizon_;
  // 控制启发式函数中时间成本的权重
  double lambda_heu_;
	// 最长的路径  检查路径点的数目
  int allocate_num_, check_num_;
  // 是否打破中断
  double tie_breaker_;
  // 是否要优化
  bool optimistic_;
  // 是否使用esdf地图
  bool is_use_esdf;
  //  控制 A* 算法的搜索分辨率，用于控制搜索空间的大小。值越小，搜索空间就越大，但搜索速度就会变慢
  // 一般是地图的分辨率
  double resolution_;
  // 地图分辨率倒数
  double inv_resolution_;
  // 时间分辨率 时间分辨率倒数
  double time_resolution_, inv_time_resolution_;
  // 时间起点
  double time_origin_;

  // 是否获取地图
  bool has_map_ = false;

  // 起点位置 地图尺寸
  Eigen::Vector2d origin_, map_size_2d_;

  // 定义起点速度 终点速度 起点加速度
  Eigen::Vector2d start_vel_, end_vel_, start_acc_ ,end_point_;
  Eigen::Matrix<double, 4, 4> phi_;  // state transit matrix
  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

	/**
	 * @brief  Given an index... compute the associated map coordinates
	 * @param  index The index
	 * @param  mx Will be set to the x coordinate
	 * @param  my Will be set to the y coordinate
	 */
	inline void indexToCells(int index, int &mx, int &my) const
	{
		my = index / X_SIZE;
		mx = index - (my * X_SIZE);
	}

  // 计算从state1 到state2的运动轨迹
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);

  // 时间转整形
  int timeToIndex(double time);
	// 获取系统时间 精确到ns
	uint64_t getSystemNSec(void);
  // 地图转世界坐标系
	Vector2d mapToWorld(Vector2i mapt) const;
  // 世界转地图坐标系
	Vector2i worldToMap(Vector2d worldpt) const;

  // 启发式函数
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);
  // 用于求解四次多项式方程的解
  vector<double> quartic(double a, double b, double c, double d, double e);
  // 求解三次多项式方程的解
  vector<double> cubic(double a, double b, double c, double d);
  // 状态转移方程
  void stateTransit(Eigen::Matrix<double, 4, 1>& state0, 
                    Eigen::Matrix<double, 4, 1>& state1,
                    Eigen::Vector2d um, double tau);

	// 判断点是否被占据的具体实现形式
	inline bool isOccupied(const int &idx, const int &idy) const;
  // 判断点是否被占据的具体实现形式 
  // 输入参数世界坐标系
  inline bool isOccupied(const double &dx, const double &dy) const;
  
protected:
	bool initialized_;

	bool startIsOccupied = false, endIsOccupied = false;

	double origin_x;
	double origin_y;

	int X_SIZE;
	int Y_SIZE;

	double _weight_g; // g数值的权重
	double _weight_h; // h数值的权重
	int _g_distance;	// g使用的距离计算方法
	int _h_distance;	// h使用的距离计算方法

	double world_distance; // 世界坐标系下的距离长度

	Eigen::Vector2i startpoint; // 开始节点
	Eigen::Vector2i endpoint;		// 终止节点
	int32_t *data;

	Eigen::Vector2i goalIdx;	 // 目标点向量坐标

	HybridGridNodePtr terminatePtr; // 终止点
	std::multimap<double, HybridGridNodePtr> openSet;

	/**
	 * @brief  Given two map coordinates... compute the associated index
	 * @param mx The x coordinate
	 * @param my The y coordinate
	 * @return The associated index
	 */
	inline int getIndex(int mx, int my) const
	{
		return my * X_SIZE + mx;
	}

public:
	Hybrid_astar();
	~Hybrid_astar();

	// 启发函数计算
	int flagHeu;					// 用于查询当前使用的路径搜索方法
	bool findFlag = true; // 检测路径搜索的结果

  // 因为该Hybrid_astar算法是可以设定搜索深度的，所以获取到的是搜索深度的终点，用于局部规划
  Eigen::Vector2d searchEndpoint; //扫描的终点

  // 接近规划边界范围 到达终点  没有路径 接近终点 在障碍物中
  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 , IN_OCCUPIED = 5};

  // 回溯路径
  void retrievePath(HybridGridNodePtr end_node);
  // 查看访问节点 
  std::vector<HybridGridNodePtr> getVisitedNodes(void);

	void resetGrid(HybridGridNodePtr ptr);																				 // 网格复位
	void resetUsedGrids();																									 // 复位使用过的网格

	void InitMap(double _resolution, double _originx, double _originy, 
                int _x_size, int _y_size, std::vector<int32_t> _mapData); 

	// 用于设定相关参数，一定要放到代码初始化之前
	// g数值的权重
	// h数值的权重
	// g_distance 的计算方法 0 1 2 3方法 欧氏距离 曼哈顿 切比学夫 对角线
	// h_distance 的计算方法
	void InitParams(ros::NodeHandle &nh);

  // 搜索函数 
  // start_pt 起点位置
  // start_v 起点速度
  // start_a 起点加速度
  // end_pt  终点位置
  // end_v   终点速度
  // init    初始化是否
  // dynamic 是否考虑运动学模型
  // time_start 开始时间
  int search(Eigen::Vector2d start_pt, Eigen::Vector2d start_v, Eigen::Vector2d start_a,
              Eigen::Vector2d end_pt, Eigen::Vector2d end_v, 
              bool init, bool dynamic = false, double time_start = -1.0);

	bool StartOrEndValidCheck(Eigen::Vector2d startpoint, Eigen::Vector2d endpoint);

  // 获取获取动力学跟踪轨迹
  std::vector<Eigen::Vector2d> getKinoTraj(double delta_t);
  void getSamples(double& ts, vector<Eigen::Vector2d>& point_set,
                              vector<Eigen::Vector2d>& start_end_derivatives);

  Eigen::MatrixXd getSamples(double &ts, int &K);

  bool CheckPathSafe(vector<Eigen::Vector2d> path); //检测路径是否安全

  // 对于已经搜索过得进行复位防止干扰下次搜索
  void reset(void);

  typedef shared_ptr<Hybrid_astar> Ptr;
};

#endif // __HYBRID_ASTAR_H
