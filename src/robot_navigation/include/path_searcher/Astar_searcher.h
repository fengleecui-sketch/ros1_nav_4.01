#ifndef __ASTART_SEARCHER_H
#define __ASTART_SEARCHER_H

#include "iostream"
#include "ros/ros.h"
#include "ros/console.h"
#include "Eigen/Eigen"
#include "node.h"
#include "algorithm"
#include "vector"

using namespace std;
using namespace Eigen;


#define Euclidean 0 // 欧式距离
#define Manhattan 1 // 曼哈顿距离
#define L_infty 2		// 切比雪夫距离
#define Diagonal 3	// 对角线
#define use_Tie_breaker 1

// #define OBSTACLE 100 // 障碍物代表数值100

// 最后打印显示的参数
typedef struct parmsLast
{
	double time;
	double dist;
	double lastTime;
	double lastDist;
} _defineParmsLast;

// Astar路径规划类
class AstarPathFinder
{
private:
	// 判断点是否被占据的具体实现形式
	bool isOccupied(const int &idx, const int &idy) const;
	// 获取路径点的ESDF数值
	int getESDFvalue(int idx,int idy);
protected:
	double resolution_;
	double g_value; // g数值的权重
	double h_value; // h数值的权重
	int g_method;	// g使用的距离计算方法
	int h_method;	// h使用的距离计算方法

	bool initialized_;

	bool startIsOccupied = false, endIsOccupied = false;

	double origin_x;
	double origin_y;

	int X_SIZE;
	int Y_SIZE;

	int OBSTACLE = 0;

	double world_distance; // 世界坐标系下的距离长度

	Eigen::Vector2i startpoint; // 开始节点
	Eigen::Vector2i endpoint;		// 终止节点

	vector<Vector2i> globalpath;	//栅格地图对应的路径

	int32_t *data;

	GridNodePtr **GridNodeMap; // 2D地图
	Eigen::Vector2i goalIdx;	 // 目标点向量坐标

	_defineParmsLast ParamsLast;

	GridNodePtr terminatePtr; // 终止图
	std::multimap<double, GridNodePtr> openSet;

	double getHeu(GridNodePtr node1, GridNodePtr node2); // 启发式获取
	double getGhi(GridNodePtr node1, GridNodePtr node2);

	void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> &neighborPtrSets, std::vector<double> &edgeCostSets, std::vector<double> &aclDist); // Astart规划获取

	unsigned char getCost(int mx, int my) const;
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
	// 获取系统时间 精确到ns
	uint64_t getSystemNSec(void);
	void mapToWorld(int mx, int my, double &wx, double &wy) const;
	bool worldToMap(double wx, double wy, int &mx, int &my) const;

	// 计算两个点之间的长度欧氏距离
	double calPointLength(Vector2d vector1,Vector2d vector2)
	{
		return (sqrt((vector1[0]-vector2[0])*(vector1[0]-vector2[0])+(vector1[1]-vector2[1])*(vector1[1]-vector2[1])));
	}

public:
	AstarPathFinder();
	~AstarPathFinder(){};

	vector<Vector2d> allpath;

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 , IN_OCCUPIED = 5};

	// 启发函数计算
	int flagHeu;					// 用于查询当前使用的路径搜索方法
	bool findFlag = true; // 检测路径搜索的结果
	bool is_use_esdf = false;	//是否使用人工势场

	double search_radius;

	bool AstarSearchResult(void); // 获取搜索路径结果
	// A*路径搜索，直接放入世界坐标系下的坐标即可进行搜索
	int AstarWorldSearch(Eigen::Vector2d startpoint, Eigen::Vector2d endpoint);
	int AstarGraphSearch(Eigen::Vector2i start_pt, Eigen::Vector2i end_pt); // A*路径搜索
	void resetGrid(GridNodePtr ptr);																				 // 网格复位
	void resetUsedGrids();																									 // 复位使用过的网格

	void InitMap(double _resolution, double _originx, double _originy, int _x_size, int _y_size, std::vector<int32_t> _mapData);

	void getPath(void);					// 获取路径
	std::vector<Eigen::Vector2d> getWorldPath(void);		// 获取世界路径，最终在世界坐标系下的路径
	double getWorldPathLength(void);		//获取总的路径点的长度
	std::vector<Eigen::Vector2i> getVisitedNodes(void); // 获取访问节点
	int getVisitedNodesNum(void);				//获取访问节点总数
	// 获得访问结点直接输出到世界坐标系
	std::vector<Eigen::Vector2d> getVisitedWorldNodes(void);

	Vector2d getTerminate(void);

	// 用于设定相关参数，一定要放到代码初始化之前
	// g数值的权重
	// h数值的权重
	// g_distance 的计算方法 0 1 2 3方法 欧氏距离 曼哈顿 切比学夫 对角线
	// h_distance 的计算方法
	void setParams(double wight_g, double wight_h, int g_distance, int h_distance,double s_radius = 100);

	// 用于传参
	void InitParams(ros::NodeHandle &nh,bool _is_use_esdf = false);

	void GetPath_ESDFvalue(int &sum_esdf,double &ave_esdf);
	bool StartOrEndValidCheck(Eigen::Vector2d startpoint, Eigen::Vector2d endpoint);

	// 提取路径采样点
	vector<Vector2d> getSamples(double distance);

	// 智能指针
  typedef shared_ptr<AstarPathFinder> Ptr;
};

#endif //__ASTART_SEARCHER_H
