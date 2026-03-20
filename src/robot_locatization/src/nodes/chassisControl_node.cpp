#include "chassis_control/chassisControl.h"

int main(int argc, char **argv)
{
	// 设置编码方式为UTF-8
	setlocale(LC_ALL, "");

	ros::init(argc, argv, "chassis_control_node"); // 节点的名字
	chassisControl chassisCtl;

	ros::spin();
	return 0;
}


