/*
 * @Author: your name
 * @Date: 2023-04-27 19:53:40
 * @LastEditTime: 2023-07-03 12:39:48
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_locatization/src/nodes/carto_locatization_node.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "cartographer_local/carto_locatization.h"

int main(int argc, char **argv)
{
	// 设置编码方式为UTF-8
	setlocale(LC_ALL, "");

	ros::init(argc, argv, "carto_locatization_node"); // 节点的名字
	carto_locatization carto_test;

	ros::spin();
	return 0;
}