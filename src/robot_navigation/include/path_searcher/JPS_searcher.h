/*
 * @Author: your name
 * @Date: 2023-07-19 20:02:31
 * @LastEditTime: 2023-08-21 22:08:54
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/include/path_searcher/JPS_searcher.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
/***
 * @ ┌─────────────────────────────────────────────────────────────┐
 * @ │┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐│
 * @ ││Esc│!1 │@2 │#3 │$4 │%5 │^6 │&7 │*8 │(9 │)0 │_- │+= │|\ │`~ ││
 * @ │├───┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴───┤│
 * @ ││ Tab │ Q │ W │ E │ R │ T │ Y │ U │ I │ O │ P │{[ │}] │ BS  ││
 * @ │├─────┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴─────┤│
 * @ ││ Ctrl │ A │ S │ D │ F │ G │ H │ J │ K │ L │: ;│" '│ Enter  ││
 * @ │├──────┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴────┬───┤│
 * @ ││ Shift  │ Z │ X │ C │ V │ B │ N │ M │< ,│> .│? /│Shift │Fn ││
 * @ │└─────┬──┴┬──┴──┬┴───┴───┴───┴───┴───┴──┬┴───┴┬──┴┬─────┴───┘│
 * @ │      │Fn │ Alt │         Space         │ Alt │Win│   HHKB   │
 * @ │      └───┴─────┴───────────────────────┴─────┴───┘          │
 * @ └─────────────────────────────────────────────────────────────┘
 * @
 * @Author: your name
 * @Date: 2022-12-04 14:16:29
 * @LastEditTime: 2023-01-06 19:28:27
 * @LastEditors: your name
 * @Description:
 * @FilePath: /tianbot_mini/src/astar_super/include/path_searcher/JPS_searcher.h
 * @可以输入预定的版权声明、个性签名、空行等
 */

#ifndef __JPS_SEARCHER_H
#define __JPS_SEARCHER_H

#include "Astar_searcher.h"
#include "JPS_utils.h"

// 定义类在跳点搜索基础上使用A*
class JPSPathFinder : public AstarPathFinder
{
private:
  _defineParmsLast JPSLastParams;
  bool isOccupied(const int &idx_x, const int &idx_y) const; // 用于判断点是否被占用
  bool isOccupied(const Eigen::Vector2i &index) const;
  bool isFree(const int &idx_x, const int &idx_y) const; // 用于判断点是否是自由的
  bool isFree(const Eigen::Vector2i &index) const;

public:
  JPS2DNeib *jn2d;

  JPSPathFinder();

  ~JPSPathFinder()
  {
    delete jn2d; // 释放
  };

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 , IN_OCCUPIED = 5};

  // 获取JPS路径
  std::vector<Eigen::Vector2i> getJPSPath();
  // 获取世界路径，最终在世界坐标系下的路径
  std::vector<Eigen::Vector2d> getJPSWorldPath(void);

  void JPSGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> &neighborPtrSets, std::vector<double> &edgeCostSets, std::vector<double> &aclDist); // 跳点搜索获取成功
  bool hasForced(const Eigen::Vector2i &idx, const Eigen::Vector2i &dir);                                                                              // 是否已经强制
  bool jump(const Eigen::Vector2i &curldx, const Eigen::Vector2i &expDir, Eigen::Vector2i &neildx);

  // JPS路径搜索，直接放入世界坐标系下的坐标即可进行搜索
  int JPSWorldSearch(Eigen::Vector2d startpoint, Eigen::Vector2d endpoint);

  /// @brief
  /// @param start_pt
  /// @param end_pt
  int JPSGraphSearch(Eigen::Vector2i start_pt, Eigen::Vector2i end_pt); // 跳点路径搜索

	// 用于传参
	void InitParams(ros::NodeHandle &nh);

  typedef shared_ptr<JPSPathFinder> Ptr;
};

#endif
