/*
 * @Author: your name
 * @Date: 2023-12-12 11:31:35
 * @LastEditTime: 2024-04-10 16:14:12
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/include/path_optimization/fast_security.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __FAST_SECURITY_H
#define __FAST_SECURITY_H

#include "iostream"
#include "ros/ros.h"
#include "Eigen/Eigen"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "bezier_curve.h"
#include "math_tools/curve_fitting.h"

#include "fstream"

#define OCCUPIED 100  //根据ROS中的地图消息格式，最大数值是100即为障碍物
#define VISITED 1     //被访问的设置为1
#define PI 3.1415926

// 宏函数定义：限制数值在指定范围内
#define LIMIT_VALUE(value, minValue, maxValue) \
    ((value) < (minValue) ? (minValue) : ((value) > (maxValue) ? (maxValue) : (value)))

using namespace std;
using namespace Eigen;
using namespace math_tools;

class Fast_Security
{
private:
  // 连续部分系数
  // 连续部分距离势场系数
  double Kcontinu_esdf; 
  // 连续部分到上一个点距离代价系数
  double Kcontinu_ldis;
  // 连续部分到终点距离代价系数
  double Kcontinu_edis;

  // 间断部分系数
  // 间断部分距离势场系数
  double Kinterru_esdf;
  // 间断部分到上一个点距离代价系数
  double Kinterru_ldis;
  // 间断部分重合点系数
  double Kinterru_rep;
  // 间断部分到终点距离代价系数
  double Kinterru_edis;

  // 阶梯部分系数
  // 阶梯部分距离场系数
  double Kstep_esdf;
  // 阶梯部分到上一个点距离代价系数
  double Kstep_ldis;
  // 阶梯部分重合点系数
  double Kstep_rep;
  // 阶梯部分到终点距离代价系数
  double Kstep_edis;

private:
  /* data */
  Vector2d start;   // 定义起点
  Vector2d end;     // 定义终点

  int endflag;      // 用来记录到达终点的次数

  Vector2i grid_map_start;
  Vector2i grid_map_end;
  // 因为Astar规划的路径拐角是固定的,所以这里直接计算了,上一个点减去下一个点,因为八连通astar探索点就是八个方向,相当于按照方向扩展
  Vector2i pointa;
  Vector2i pointb;

  vector<Vector2d> originPath;    //原始路径
  vector<Vector2i> origingridPath;    //原始路径

  // 求向量AB
  Vector2i v_b_a; 
  // 求解与向量AB垂直的向量
  Vector2i ver_b_a;

  // 优化后的路径
  vector<Vector2d> optpath;     //优化路径

  // vector<pair<int,Vector2i>> subpoint;
  // 对路径分段
  vector<pair<int,Vector2i>> dividePath;
  // 垂直与分段路径的点集合
  vector<pair<int,Vector2i>> verdividePath;

  // 连续路径分段
  // int 代表第几段
  vector<pair<int,Vector2i>> continuDivid;
  // 垂直与连续路径分段
  // int 代表第几段落
  vector<pair<int,Vector2i>> vercontinuDivid;
  // 定义分段点
  vector<Vector2d> worldividNodes;
  vector<Vector2i> dividNodes;

  // 拐点搜索的点的集合
  vector<vector<Vector2i>> inflectPoins;
  vector<vector<Vector2d>> inflectPoinsWorld;

  vector<int> continu_add_search; //连续段向上搜索最小值
  vector<int> continu_min_search; //连续段向上搜索最小值

  vector<int> inflect_add_search; //间断段连续向上搜索最小值
  vector<int> inflect_min_search; //间断段连续向上搜索最小值

  Vector2i nowvec; //当前向量
  Vector2i lastvec; //上一个向量

  // 用于存储当前节点是否扩展
  vector<pair<bool,Vector2i>> inflectExten;
  // 拐点集合
  vector<Vector2i> inflectAssem;

  // 定义标志位容器判断哪一段可以添加进来
  vector<int> flagOK;
  // 连续点集合
  vector<Vector2i> continuAssem;
  // 过b的向量
  vector<Vector2i> vercontAssem;
  // 连续搜索的点的集合
  vector<vector<Vector2i>> continuPoints;
  vector<vector<Vector2d>> continuPointsWorld;

  // 连续点每段连续数量计数
  vector<int> continuNum;
  // 从连续点中进行采样
  vector<vector<Vector2i>> continuSample;
  vector<vector<Vector2d>> continuSampleWorld;
  // 未进行优化的路径
  vector<Vector2d> notoptpath;

  // 提取最终优化的采样点集合
  vector<pair<int,vector<Vector2i>>> optimiPoints;
  vector<vector<Vector2d>> optimiPointsWorld;

  vector<vector<Vector2d>> connectPointsWorld;

  // 用于显示访问到的优化的点
  vector<Vector2d> visualOptpoints;

  // 需要优化的路径
  vector<vector<Vector2d>> needoptpath;
  // 需要分段的路径
  vector<Vector2i> needivipath;
  // 对路径点进行插值
  vector<Vector2i> interpolation_path;
  // 连续段的搜索点集合
  vector<pair<int,vector<Vector2i>>> dividContinuPoints;
  vector<pair<int,vector<Vector2d>>> dividContinuPointsWorld;

  vector<Vector2i> frontVec;    //前点集合
  vector<Vector2i> afterVec;    //后点集合

  vector<Vector2i> frontToafterVec;   //由前向后的向量集合
  vector<Vector2i> afterTofrontVec;   //由后向前的向量集合
  vector<Vector2i> verfrontToafterVec;   //垂直向量的集合
  vector<int> max_step_vec;           //最大步长度集合

  double resolution;// 定义搜索分辨率
  double resolution_inv;  //搜索分辨率的倒数
  double search_length;   //定义搜索长度
  double search_num;     //定义搜索点数目
  double esdf_value;    //人工势场数值

  double origin_x;  //地图的x起点Vector2d
  double origin_y;  //地图的y起点
  double mapover_x; //地图的x终点
  double mapover_y; //地图的y终点

  // 地图的x轴方向的栅格地图尺寸
  int grid_map_x;
  int grid_map_y;

  // 当前的势场地图
	int8_t *mapdata;
  // 用一位数组来索引格点是否探索过，防止重复探索
  vector<int> search_data;

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
  Vector2d mapToWorld(Vector2i mapt) const;
  /**
   * @brief 世界坐标系转栅格地图坐标系
   * @param wx   世界坐标x
   * @param wy   世界坐标y        if(isOccupied(pointS))
        {
          break;
        }
   * @param mx   地图坐标x
   * @param my   地图坐标y
   * @return
   * @attention
   * @todo
   * */
  Vector2i worldToMap(Vector2d worldpt) const;
  // 判断是否在地图中
  bool isInMap(Eigen::Vector2d worldpt);
	// 判断点是否被占据的具体实现形式
	bool isOccupied(const Vector2d worldpt) const;
  // 判断点是否被占据的具体实现形式
  bool isOccupied(const Vector2i gridpt) const;
  // 设定访问过的点为1
  void setVisited(const Vector2i gridpt);
  // 判断点是否重复
  bool isRepeated(const Vector2i gridpt) const;
  // 获取访问的次数
  int getVisitedNum(const Vector2i gridpt) const;
  // 获取当前点的势场数值
  int8_t getESDFvalue(Vector2i gridpt);
  // 判断向量是否共线
  bool isCollinear(Vector2i vector1,Vector2i vector2,bool strict);
  // 判断向量是否共线
  bool isCollinear(Vector2d vector1,Vector2d vector2,bool strict);
  // 计算单位向量
  Vector2d calUnitvector(Vector2i unitv);
  // 计算单位向量
  Vector2d calUnitvector(Vector2d unitv);
  // 计算向量长度
  Vector2d calVectorlength(Vector2d v);
  // 计算两个点之间的长度欧氏距离
  double calPointLength(Vector2i vector1,Vector2i vector2);
  // 计算两个点之间的长度欧氏距离
  double calPointLength(Vector2d vector1,Vector2d vector2);
  // 计算向量之间的夹角
  double calVectorAngle(Vector2i vector1,Vector2i vector2);
  // 计算向量之间的夹角
  double calVectorAngle(Vector2d vector1,Vector2d vector2);
  // 计算向量之间的乘积
  double calVectorProduct(Vector2i vector1,Vector2i vector2);

  // 搜索障碍物
  // searchLength 搜索长度
  // point 扩展起点
  // vertical_m 垂直平分线
  // add_num 向上扩展截止
  // min_num 向下扩展截止
  void Search_Obstacle(double searchLength,Vector2i point,Vector2i vertical_m,int &add_num,int &min_num);
  // 节点扩展
  // addsearch 累加搜索长度
  // minsearch 累减搜索长度
  // point 扩展起点
  // vertical_m 垂直平分线
  // pointExtension 返回的点集合
  void Node_Extension(int addsearch,int minsearch,Vector2i point,Vector2i vertical_m,vector<Vector2i> &pointExtension);
  // 节点扩展但是方形扩展
  // addsearch 累加搜索长度
  // minsearch 累减搜索长度
  // point 扩展起点
  // pointExtension 返回的点集合
  void Rectangle_Extension(int addsearch,int minsearch,Vector2i point,vector<Vector2i> &pointExtension);
  // 中断节点扩展
  // searchLength 搜索长度
  // point 扩展起点
  // vertical_m 垂直平分线
  // pointExtension 返回的点集合
  void Inflect_Extension(int test,vector<Vector2i> &front1Extension,vector<Vector2i> &front2Extension,
vector<Vector2i> &after1Extension,vector<Vector2i> &after2Extension);
  // 连续点扩展
  // pointExtension 返回的点集合
  void Continu_Extension(vector<vector<Vector2i>> &pointsMap,vector<vector<Vector2d>> &pointsWorld);
  // 连续扩展部分提取
  void ContinuExtension_Extract(vector<vector<Vector2i>> &continu_sample);
  // 中断区域扩展
  // pointsMap 栅格地图返回的扩展点
  // pointsWorld 世界地图返回的扩展点
  void Interrupt_Extension(vector<vector<Vector2i>> &pointsMap,vector<vector<Vector2d>> &pointsWorld);
  // 路径点采样
  void SampleGridPath(vector<Vector2i> gridpath);
  // 搜索扩展限制点
  // add_search_vec 向上扩展容器
  // min_search_vec 向下扩展容器
  void FindContinueLimitExten(vector<int> &add_search_vec,vector<int> &min_search_vec);
  // 搜索扩展限制点
  // add_search_vec 向上扩展容器
  // min_search_vec 向下扩展容器
  void FindInflectLimitExten(int test);
  // 计算分段点
  // dividMap 栅格地图分段点
  // dividWorld 世界地图分段点
  void CalDivideNodes(vector<Vector2i> &dividMap,vector<Vector2d> &dividWorld);
  // 查询间断点的数目
  // 这里的间断点是中断扩展和连续扩展都没包含的点
  // gridPath 栅格路径点
  // firstpoint 输入的第一个点
  // secondpoint 输入的第二个点
  // firstnum 输出的第一个点的位置
  // secondnum 输出的第二个点的位置
  void CheckInflectNum(vector<Vector2i> gridPath,Vector2i firstpoint,Vector2i secondpoint,int &firstnum,int &secondnum);
  // 输出路径点前后共线段的坐标位置集合
  // gridPath 栅格路径点
  vector<int> CollinearPosition(vector<Vector2i> gridPath);
  // 对路径进行分段
  // gridPath 栅格路径点
  // dividvec 用于分段的数组
  // dividpath 分段出来的栅格路径点
  void DividPath(vector<Vector2i> gridPath,vector<int> dividvec,vector<vector<Vector2d>> &dividpath);
  // 将世界坐标点转换成栅格坐标系下的整数点
  vector<Vector2i> worldPathToGridPath(vector<Vector2d> worldPath);
  // 将栅格坐标系下的整数点转换成世界坐标系下的点
  vector<Vector2d> GridPathToWorldPath(vector<Vector2i> gridPath);
  // 检查点之间有没有障碍物
  // firstpoint  第一个点
  // secondpoint 第二个点
  bool CheckObstaclePoints(Vector2i firstpoint,Vector2i secondpoint);
  // 对路径按照固定间距分段
  // step 分段长度
  // oldpath 原始路径
  // dividpath 分段之后的路径
  void Setlength_DividPath(double step,vector<Vector2i> oldpath,vector<Vector2i> &newpath);
  // 对路径进行优化
  vector<Vector2d> PathOptimization(vector<vector<Vector2i>> continu,vector<vector<Vector2i>> inflect);
  // 使用贝塞尔曲线对路径进行优化
  vector<Vector2d> BezierPathOpt(vector<vector<Vector2d>> needoptpath);
  // 对路径进行插值，这里是按照距离等距插值
  // length 插值距离
  // oldpath 原来路径
  // interpolatpath 插值路径
  void Setlength_Interpolation_Path(double length,vector<Vector2i> oldpath,vector<Vector2i> &newpath);

  void visual_VisitedNode(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> visitnodes,
float a_set,float r_set,float g_set,float b_set,float length);

  // 可视化采样点集合
  void visual_SamplesNode(ros::Publisher pathPublish, vector<vector<Vector2d>> samplenodes,
float a_set,float r_set,float g_set,float b_set,float k_length);

  // 使用贝塞尔进行优化
  BEZIER::Ptr bezier_opt;

  // 用于对多项式求解求导
  curve_fitting curve;

  // 用于保存需要的数据
  ofstream file;
private:
  ros::Publisher visitNodesPub;  //访问节点发布节点
  ros::Publisher visitOptpathPub;  //优化的路径点发布
  ros::Publisher sampleNodesPub;    //采样点初始化
  ros::Publisher continuNodesPub;    //连续点初始化
  ros::Publisher suppleMentPub;     //补充发布

public:
  // 设定地图参数
  void SetMapParams(double resolution_,double origin_x_,double origin_y_,
                    int map_x_size,int map_y_size,std::vector<int8_t> _mapData);

  // 用于传参
  void InitParams(ros::NodeHandle &nh);

  // 复位访问的节点
  void resetVisited(void);

  // 用于查看各种节点路径可视化
  void visualNodes(void);

  // 以等腰三角形的底边中点到顶点的向量作为搜索线段，在该线段上搜索人工势场最小值作为
  // 优化后的路径点，最终形成一条优化后的曲线
  // 参数是原来读经
  vector<Vector2d> Fast_Security_Search(vector<Vector2d> oripath);

  // 获取优化后的路径的欧几里得距离场信息数据
  void GetPathESDFvalue(int &sum_esdf,double &ave_esdf,int &path_num);

  // 重规划判断执行
  // vector<Vector2d> Fast_Security_Replan(vector<Vector2d> oripath);

  // 获取走廊访问的节点
  int getCorridorVisitedNum(void);

  // 获取最终优化过的路径的长度
  double getOptedPathLength(void);

  // 已经被优化的最终的路径
  vector<Vector2d> optedpath;

	// 智能指针
  typedef shared_ptr<Fast_Security> Ptr;

  // 定义完成标志位，用于记录每次完成的进度，查看到多少
  int completed_flag;

  Fast_Security(/* args */);
  ~Fast_Security();

};


#endif


