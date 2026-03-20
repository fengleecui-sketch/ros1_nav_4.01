/*
 * @Author: your name
 * @Date: 2023-04-27 19:53:40
 * @LastEditTime: 2023-05-06 12:52:41
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /Robot_Formation/src/robot_locatization/src/nodes/plicp_odometry_node.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "pl_icp/plicp_odometry.h"

int main(int argc, char **argv)
{
	// 设置编码方式为UTF-8
	setlocale(LC_ALL, "");

	ros::init(argc, argv, "scan_match_plicp_node"); // 节点的名字
	ScanMatchPLICP scan_match_plicp;

	ros::spin();
	return 0;
}
