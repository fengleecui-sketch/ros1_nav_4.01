/*
 * @Author: your name
 * @Date: 2023-07-19 20:02:31
 * @LastEditTime: 2023-08-11 20:48:41
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_costmap/src/nodes/local_map_deal_node.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "map_deal/map_deal.h"
#include "map_deal/signed_distance_field_2d.h"

using namespace map_deal;
using namespace std;
using namespace Eigen;


// 测试距离场
void TestSignedDistance(void){

  std::array<int, 2> size{800, 800};

  GridMap2D<uint8_t> grid_map;
  grid_map.set_cell_number(size);
  grid_map.set_origin(std::array<double, 2>{0.0, 0.0});
  grid_map.set_resolution(std::array<double, 2>{0.05, 0.05});
  grid_map.FillCircle(Eigen::Vector2d(10, 10), 5);
  grid_map.FillConvexPoly(
      vector_Eigen<Vector2d>{Vector2d(30, 15), Vector2d(40, 15),
                       Vector2d(40, 20), Vector2d(30, 20)});
  grid_map.FillPoly(vector_Eigen<Vector2d>{Vector2d(0, 20), Vector2d(20, 20),
                                     Vector2d(25, 25), Vector2d(15, 22),
                                     Vector2d(0, 30)});

  cv::imshow("grid map", grid_map.BinaryImage());

  SignedDistanceField2D sdf(std::move(grid_map));
  auto t0 = chrono::high_resolution_clock::now();

  sdf.UpdateSDF();
  auto t1 = chrono::high_resolution_clock::now();

  double total_ms =
      chrono::duration_cast<chrono::microseconds>(t1 - t0).count() / 1000.0;

  cout << "time for 800x800 sdf: " << total_ms << " ms" << endl;
  cv::imshow("sdf", sdf.esdf().ImageSec());
  cv::waitKey(0);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "deal_local_map");

  // 发布话题
  deal_all_map dealmaptest;

  // TestSignedDistance();

  ros::spin();
  return 0;
}
