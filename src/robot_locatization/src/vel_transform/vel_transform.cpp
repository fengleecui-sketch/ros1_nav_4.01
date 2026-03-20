#include "vel_transform/vel_transform.h"

namespace vel_transform
{
// 全局速度转局部
void GlobalVelocityToLocal(Odom_data_define *localVelocity, Odom_data_define *globalVelocity)
{
	if (abs(globalVelocity->Vx) < 0.005)
	{
		globalVelocity->Vx = 0;
	}
	if (abs(globalVelocity->Vy) < 0.005)
	{
		globalVelocity->Vy = 0;
	}

	// 速度解算
	double velocity_vector;		// 合成速度
	uint8_t quadrant_yaw = 0; // 定义车体所在象限 1-4
	uint8_t quadrant_vel = 0; // 车的局部速度合成
	velocity_vector = sqrt(pow(globalVelocity->Vx, 2) + pow(globalVelocity->Vy, 2));
	// 根据yaw轴角度判断当前车的姿态在第几象限
	if (localVelocity->yaw >= 0 && localVelocity->yaw <= M_PI / 2)
	{
		quadrant_yaw = 1;
	}
	else if (localVelocity->yaw > M_PI / 2 && localVelocity->yaw <= M_PI)
	{
		quadrant_yaw = 2;
	}
	else if (localVelocity->yaw > -M_PI && localVelocity->yaw <= -M_PI / 2)
	{
		quadrant_yaw = 3;
	}
	else if (localVelocity->yaw > -M_PI / 2 && localVelocity->yaw < 0)
	{
		quadrant_yaw = 4;
	}

	// 全局速度根据Vx 和 Vy的正负分四种情况
	// x+ y+
	if (globalVelocity->Vx >= 0 && globalVelocity->Vy >= 0)
	{
		quadrant_vel = 1;
	}
	// x- y+
	else if (globalVelocity->Vx < 0 && globalVelocity->Vy >= 0)
	{
		quadrant_vel = 2;
	}
	// x- y-
	else if (globalVelocity->Vx < 0 && globalVelocity->Vy < 0)
	{
		quadrant_vel = 3;
	}
	// x+ y-
	else if (globalVelocity->Vx >= 0 && globalVelocity->Vy < 0)
	{
		quadrant_vel = 4;
	}

	// cout<<(uint16_t)quadrant_yaw<<(uint16_t)quadrant_vel<<endl;

	// 判断车的姿态在第几象限
	switch (quadrant_yaw)
	{
	case 1:	//第一象限
		if(quadrant_vel == 1)	// 全局速度 x+ y+
		{
			if (globalVelocity->Vx == 0)
			{
				localVelocity->Vx = velocity_vector*sin(localVelocity->yaw);
				localVelocity->Vy = velocity_vector*cos(localVelocity->yaw);
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = localVelocity->yaw - globalVelocity->yaw;
				// 解算局部速度
				localVelocity->Vx = velocity_vector*cos(localVelocity->theta);
				localVelocity->Vy = -velocity_vector*sin(localVelocity->theta);
			}
		}
		else if(quadrant_vel == 2) //全局速度 x- y+
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = velocity_vector*sin(localVelocity->yaw);
				localVelocity->Vy = velocity_vector*cos(localVelocity->yaw);
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = localVelocity->yaw + abs(globalVelocity->yaw);
				// 解算局部速度
				localVelocity->Vx = -velocity_vector*cos(localVelocity->theta);
				localVelocity->Vy = velocity_vector*sin(localVelocity->theta);
			}
		}
		else if(quadrant_vel == 3)	//全局速度 x- y-
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = -velocity_vector*sin(localVelocity->yaw);
				localVelocity->Vy = -velocity_vector*cos(localVelocity->yaw);
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = localVelocity->yaw - globalVelocity->yaw;
				// 解算局部速度
				localVelocity->Vx = -velocity_vector*cos(localVelocity->theta);
				localVelocity->Vy = velocity_vector*sin(localVelocity->theta);
			}
		}
		else if(quadrant_vel == 4)	//全局速度 x+ y-
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = -velocity_vector*sin(localVelocity->yaw);
				localVelocity->Vy = -velocity_vector*cos(localVelocity->yaw);
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = M_PI/2 - localVelocity->yaw - abs(globalVelocity->yaw);
				// 解算局部速度
				localVelocity->Vx = -velocity_vector*sin(localVelocity->theta);
				localVelocity->Vy = -velocity_vector*cos(localVelocity->theta);
			}
		}
		break;
	case 2:		//第二象限
		if(quadrant_vel == 1)		//全局速度x+ y+
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = cos(localVelocity->yaw-M_PI/2);
				localVelocity->Vy = -sin(localVelocity->yaw-M_PI/2);
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = globalVelocity->yaw-localVelocity->yaw+M_PI/2;
				//解算局部速度
				localVelocity->Vx = velocity_vector*sin(localVelocity->theta);
				localVelocity->Vy = -velocity_vector*cos(localVelocity->theta);
			}
		}
		else if(quadrant_vel == 2)	//全局速度x- y+
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = cos(localVelocity->yaw-M_PI/2);
				localVelocity->Vy = -sin(localVelocity->yaw-M_PI/2);
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = M_PI - localVelocity->yaw - abs(globalVelocity->yaw);
				// 解算局部速度
				localVelocity->Vx = velocity_vector*cos(localVelocity->theta);
				localVelocity->Vy = velocity_vector*sin(localVelocity->theta);
			}
		}
		else if(quadrant_vel == 3)	//全局速度x- y-
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = -cos(localVelocity->yaw-M_PI/2);
				localVelocity->Vy = sin(localVelocity->yaw-M_PI/2);				
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = M_PI + localVelocity->yaw + globalVelocity->yaw;
				// 解算局部速度
				localVelocity->Vx = velocity_vector*sin(localVelocity->theta);
				localVelocity->Vy = velocity_vector*cos(localVelocity->theta);
			}
		}
		else if(quadrant_vel == 4)	//全局速度 x+ y-
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = -cos(localVelocity->yaw-M_PI/2);
				localVelocity->Vy = sin(localVelocity->yaw-M_PI/2);				
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = M_PI - localVelocity->yaw - abs(globalVelocity->yaw);
				// 解算局部速度
				localVelocity->Vx = -velocity_vector*cos(localVelocity->theta);
				localVelocity->Vy = -velocity_vector*sin(localVelocity->theta);
			}	
		}
		break;
	case 3:	//第三象限
		if(quadrant_vel == 1)		//全局速度 x+ y+
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = -velocity_vector*sin(localVelocity->yaw+M_PI);
				localVelocity->Vy = -velocity_vector*cos(localVelocity->yaw+M_PI);
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = globalVelocity->yaw - localVelocity->yaw - M_PI;
				// 解算局部速度
				localVelocity->Vx = -velocity_vector*cos(localVelocity->theta);
				localVelocity->Vy = -velocity_vector*sin(localVelocity->theta); 
			}
		}
		else if(quadrant_vel == 2)  //全局速度 x- y+
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = -velocity_vector*sin(localVelocity->yaw+M_PI);
				localVelocity->Vy = -velocity_vector*cos(localVelocity->yaw+M_PI);				
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = -localVelocity->yaw - M_PI/2 - abs(globalVelocity->yaw);
				// 解算局部速度
				localVelocity->Vx = velocity_vector*sin(localVelocity->theta);
				localVelocity->Vy = -velocity_vector*sin(localVelocity->theta);
			}
		}
		else if(quadrant_vel == 3)	//全局速度 x- y-
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = velocity_vector*sin(localVelocity->yaw+M_PI);
				localVelocity->Vy = velocity_vector*cos(localVelocity->yaw+M_PI);
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = globalVelocity->yaw - localVelocity->yaw - M_PI;
				// 解算局部速度
				localVelocity->Vx = velocity_vector*cos(localVelocity->theta);
				localVelocity->Vy = velocity_vector*sin(localVelocity->theta);
			}
		}
		else if(quadrant_vel == 4)	//全局速度 x+ y-
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = velocity_vector*sin(localVelocity->yaw+M_PI);
				localVelocity->Vy = velocity_vector*cos(localVelocity->yaw+M_PI);				
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = abs(globalVelocity->yaw) + localVelocity->yaw + M_PI;
				// 解算局部速度
				localVelocity->Vx = -velocity_vector*sin(localVelocity->theta);
				localVelocity->Vy = velocity_vector*cos(localVelocity->theta);
			}
		}
		break;
	case 4:	//第四象限
		if(quadrant_vel == 1)
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = -velocity_vector*sin(abs(localVelocity->yaw));
				localVelocity->Vy = velocity_vector*cos(abs(localVelocity->yaw));
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = globalVelocity->yaw - localVelocity->yaw - M_PI/2;
				// 解算局部速度
				localVelocity->Vx = -velocity_vector*sin(localVelocity->theta);
				localVelocity->Vy = velocity_vector*cos(localVelocity->theta);
			}
		}
		else if(quadrant_vel == 2)
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = -velocity_vector*sin(abs(localVelocity->yaw));
				localVelocity->Vy = velocity_vector*cos(abs(localVelocity->yaw));				
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = abs(localVelocity->yaw) - abs(globalVelocity->yaw);
				// 解算局部速度
				localVelocity->Vx = -velocity_vector*cos(localVelocity->theta);
				localVelocity->Vy = -velocity_vector*sin(localVelocity->theta);
			}
		}
		else if(quadrant_vel == 3)
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = velocity_vector*sin(abs(localVelocity->yaw));
				localVelocity->Vy = -velocity_vector*cos(abs(localVelocity->yaw));						
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = M_PI - abs(globalVelocity->yaw) + localVelocity->yaw;
				// 解算局部速度
				localVelocity->Vx = velocity_vector*cos(localVelocity->theta);
				localVelocity->Vy = -velocity_vector*sin(localVelocity->theta);
			}
		}
		else if(quadrant_vel == 4)
		{
			if(globalVelocity->Vx == 0)
			{
				localVelocity->Vx = velocity_vector*sin(abs(localVelocity->yaw));
				localVelocity->Vy = -velocity_vector*cos(abs(localVelocity->yaw));						
			}
			else
			{
				// 计算全局yaw
				globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
				// 合成车底盘的速度矢量角度
				localVelocity->theta = abs(localVelocity->yaw) - abs(globalVelocity->yaw);
				// 解算局部速度
				localVelocity->Vx = velocity_vector*cos(localVelocity->theta);
				localVelocity->Vy = velocity_vector*sin(localVelocity->theta);
			}
		}
		break;
	
	default:
		break;
	}

	
}

// 换成麦轮模型之后的飘逸问题是因为获得的数据是车的局部速度数据，现在将数据转换为全局速度数据
void LocalVelocityToGlobal(Odom_data_define *localVelocity, Odom_data_define *globalVelocity)
{
	if (abs(localVelocity->Vx) < 0.005)
	{
		localVelocity->Vx = 0;
	}
	if (abs(localVelocity->Vy) < 0.005)
	{
		localVelocity->Vy = 0;
	}

	// 速度解算
	double velocity_vector;		// 合成速度
	uint8_t quadrant_yaw = 0; // 定义车体所在象限 1-4
	uint8_t quadrant_vel = 0; // 车的局部速度合成
	velocity_vector = sqrt(pow(localVelocity->Vx, 2) + pow(localVelocity->Vy, 2));

	// 根据yaw轴角度判断当前车的姿态在第几象限
	if (localVelocity->yaw >= 0 && localVelocity->yaw <= M_PI / 2)
	{
		quadrant_yaw = 1;
	}
	else if (localVelocity->yaw > M_PI / 2 && localVelocity->yaw <= M_PI)
	{
		quadrant_yaw = 2;
	}
	else if (localVelocity->yaw > -M_PI && localVelocity->yaw <= -M_PI / 2)
	{
		quadrant_yaw = 3;
	}
	else if (localVelocity->yaw > -M_PI / 2 && localVelocity->yaw < 0)
	{
		quadrant_yaw = 4;
	}

	// 车的速度根据Vx 和 Vy的正负分四种情况
	// x+ y+
	if (localVelocity->Vx >= 0 && localVelocity->Vy >= 0)
	{
		quadrant_vel = 1;
	}
	// x- y+
	else if (localVelocity->Vx < 0 && localVelocity->Vy >= 0)
	{
		quadrant_vel = 2;
	}
	// x- y-
	else if (localVelocity->Vx < 0 && localVelocity->Vy < 0)
	{
		quadrant_vel = 3;
	}
	// x+ y-
	else if (localVelocity->Vx >= 0 && localVelocity->Vy < 0)
	{
		quadrant_vel = 4;
	}

	// ROS_INFO("yaw:%d   vel:%d    ",quadrant_yaw,quadrant_vel);

	// 对每个象限下的四种情况进行处理
	switch (quadrant_yaw)
	{
	case 1:
		if (quadrant_vel == 1) // x+ y+
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = -velocity_vector * sin(globalVelocity->yaw);
				globalVelocity->Vy = velocity_vector * cos(globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->theta + localVelocity->yaw;
				globalVelocity->Vx = velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = velocity_vector * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 2) // x- y+
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = -velocity_vector * sin(globalVelocity->yaw);
				globalVelocity->Vy = velocity_vector * cos(globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标 下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->yaw + abs(localVelocity->theta);
				globalVelocity->Vx = velocity_vector * cos(globalVelocity->yaw);
				// 加了个-号
				globalVelocity->Vy = -velocity_vector * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 3) // x- y-
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = velocity_vector * sin(globalVelocity->yaw);
				globalVelocity->Vy = -velocity_vector * cos(globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->yaw + abs(localVelocity->theta);
				globalVelocity->Vx = -velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = -velocity_vector * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 4) // x+ y-
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = velocity_vector * sin(globalVelocity->yaw);
				globalVelocity->Vy = -velocity_vector * cos(globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->yaw + localVelocity->theta;
				globalVelocity->Vx = velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = velocity_vector * sin(globalVelocity->yaw);
			}
		}
		break;
	case 2:
		if (quadrant_vel == 1)
		{
			if (localVelocity->Vx == 0) // x+ y+
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = -velocity_vector * cos(globalVelocity->yaw - M_PI / 2);
				globalVelocity->Vy = -velocity_vector * sin(globalVelocity->yaw - M_PI / 2);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标 下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->yaw + localVelocity->theta;
				globalVelocity->Vx = velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = velocity_vector * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 2) // x- y+
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = -velocity_vector * cos(globalVelocity->yaw - M_PI / 2);
				globalVelocity->Vy = -velocity_vector * sin(globalVelocity->yaw - M_PI / 2);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标 下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = M_PI - localVelocity->yaw + abs(localVelocity->theta);
				globalVelocity->Vx = velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = -velocity_vector * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 3) // x- y-
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = velocity_vector * cos(globalVelocity->yaw - M_PI / 2);
				globalVelocity->Vy = velocity_vector * sin(globalVelocity->yaw - M_PI / 2);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标 下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = M_PI - localVelocity->yaw - localVelocity->theta;
				globalVelocity->Vx = velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = -velocity_vector * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 4) // x+ y-
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = velocity_vector * cos(globalVelocity->yaw - M_PI / 2);
				globalVelocity->Vy = velocity_vector * sin(globalVelocity->yaw - M_PI / 2);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标 下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->yaw + localVelocity->theta;
				globalVelocity->Vx = velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = velocity_vector * sin(globalVelocity->yaw);
			}
		}
		break;
	case 3:
		if (quadrant_vel == 1) // x+ y+
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = velocity_vector * sin(globalVelocity->yaw + M_PI);
				globalVelocity->Vy = -velocity_vector * cos(globalVelocity->yaw + M_PI);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标 下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->yaw + M_PI + localVelocity->theta;
				globalVelocity->Vx = -velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = -velocity_vector * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 2) // x- y+
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = velocity_vector * sin(globalVelocity->yaw + M_PI);
				globalVelocity->Vy = -velocity_vector * cos(globalVelocity->yaw + M_PI);
			}
			else
			{
				// 计算小车自身big_house合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标
				globalVelocity->yaw = localVelocity->yaw + M_PI + localVelocity->theta;
				globalVelocity->Vx = velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = velocity_vector * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 3) // x- y-
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = velocity_vector * sin(globalVelocity->yaw + M_PI);
				globalVelocity->Vy = -velocity_vector * cos(globalVelocity->yaw + M_PI);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标// x+ y+
				globalVelocity->yaw = localVelocity->yaw + M_PI + localVelocity->theta;
				globalVelocity->Vx = velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = velocity_vector * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 4) // x+ y-
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = -velocity_vector * sin(globalVelocity->yaw + M_PI);
				globalVelocity->Vy = velocity_vector * cos(globalVelocity->yaw + M_PI);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标
				globalVelocity->yaw = localVelocity->yaw + M_PI + localVelocity->theta;
				globalVelocity->Vx = -velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = -velocity_vector * sin(globalVelocity->yaw);
			}
		}
		break;

	case 4:
		if (quadrant_vel == 1) // x+ y+
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = velocity_vector * sin(globalVelocity->yaw + M_PI / 2);
				globalVelocity->Vy = velocity_vector * cos(globalVelocity->yaw + M_PI / 2);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标
				globalVelocity->yaw = -localVelocity->yaw - localVelocity->theta;
				globalVelocity->Vx = velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = -velocity_vector * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 2) // x- y+
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = velocity_vector * sin(-globalVelocity->yaw);
				globalVelocity->Vy = velocity_vector * cos(-globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标
				globalVelocity->yaw = -localVelocity->yaw - localVelocity->theta;
				globalVelocity->Vx = -velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = velocity_vector * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 3) // x- y-
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = -velocity_vector * sin(-globalVelocity->yaw);
				globalVelocity->Vy = -velocity_vector * cos(-globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标
				globalVelocity->yaw = -localVelocity->yaw - localVelocity->theta;
				globalVelocity->Vx = -velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = velocity_vector * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 4) // x+ y-
		{
			if (localVelocity->Vx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Vx = -velocity_vector * sin(-globalVelocity->yaw);
				globalVelocity->Vy = -velocity_vector * cos(-globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Vy / localVelocity->Vx);
				// 全局坐标
				globalVelocity->yaw = -localVelocity->yaw - localVelocity->theta;
				globalVelocity->Vx = velocity_vector * cos(globalVelocity->yaw);
				globalVelocity->Vy = -velocity_vector * sin(globalVelocity->yaw);
			}
		}
		break;
	default:
		break;
	}

	// 根据解算出来的全局速度求出全局角度
	if(globalVelocity->Vx == 0)
	{
		if(globalVelocity->Vy < 0)
		{
			globalVelocity->yaw = -M_PI/2;
		}
		else
		{
			globalVelocity->yaw = M_PI/2;
		}
	}
	else{
		// 在第一象限
		if(globalVelocity->Vx > 0 && globalVelocity->Vy > 0)
		{
			globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
		}
		// 在第四象限
		if(globalVelocity->Vx > 0 && globalVelocity->Vy < 0)
		{
			globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx);
		} 
		// 在第二象限
		else if(globalVelocity->Vx < 0 && globalVelocity->Vy > 0)
		{
			globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx) + M_PI;
		}
		// 在第三象限
		else if(globalVelocity->Vx < 0 && globalVelocity->Vy < 0)
		{
			globalVelocity->yaw = atan(globalVelocity->Vy/globalVelocity->Vx) - M_PI;
		}
	}
}

// 换成麦轮模型之后的飘逸问题是因为获得的数据是车的局部速度数据，现在将数据转换为全局速度数据
void LocalAcceleraToGlobal(Odom_data_define *localVelocity, Odom_data_define *globalVelocity)
{
	if (abs(localVelocity->Accx) < 0.005)
	{
		localVelocity->Accx = 0;
	}
	if (abs(localVelocity->Accy) < 0.005)
	{
		localVelocity->Accy = 0;
	}

	// 速度解算
	double velocity_accel;		// 合成速度
	uint8_t quadrant_yaw = 0; // 定义车体所在象限 1-4
	uint8_t quadrant_vel = 0; // 车的局部速度合成
	velocity_accel = sqrt(pow(localVelocity->Accx, 2) + pow(localVelocity->Accy, 2));

	// 根据yaw轴角度判断当前车的姿态在第几象限
	if (localVelocity->yaw >= 0 && localVelocity->yaw <= M_PI / 2)
	{
		quadrant_yaw = 1;
	}
	else if (localVelocity->yaw > M_PI / 2 && localVelocity->yaw <= M_PI)
	{
		quadrant_yaw = 2;
	}
	else if (localVelocity->yaw > -M_PI && localVelocity->yaw <= -M_PI / 2)
	{
		quadrant_yaw = 3;
	}
	else if (localVelocity->yaw > -M_PI / 2 && localVelocity->yaw < 0)
	{
		quadrant_yaw = 4;
	}

	// 车的速度根据Vx 和 Vy的正负分四种情况
	// x+ y+
	if (localVelocity->Accx >= 0 && localVelocity->Accy >= 0)
	{
		quadrant_vel = 1;
	}
	// x- y+
	else if (localVelocity->Accx < 0 && localVelocity->Accy >= 0)
	{
		quadrant_vel = 2;
	}
	// x- y-
	else if (localVelocity->Accx < 0 && localVelocity->Accy < 0)
	{
		quadrant_vel = 3;
	}
	// x+ y-
	else if (localVelocity->Accx >= 0 && localVelocity->Accy < 0)
	{
		quadrant_vel = 4;
	}

	// ROS_INFO("yaw:%d   vel:%d    ",quadrant_yaw,quadrant_vel);

	// 对每个象限下的四种情况进行处理
	switch (quadrant_yaw)
	{
	case 1:
		if (quadrant_vel == 1) // x+ y+
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = -velocity_accel * sin(globalVelocity->yaw);
				globalVelocity->Accy = velocity_accel * cos(globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->theta + localVelocity->yaw;
				globalVelocity->Accx = velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = velocity_accel * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 2) // x- y+
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = -velocity_accel * sin(globalVelocity->yaw);
				globalVelocity->Accy = velocity_accel * cos(globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标 下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->yaw + abs(localVelocity->theta);
				globalVelocity->Accx = -velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = velocity_accel * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 3) // x- y-
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = velocity_accel * sin(globalVelocity->yaw);
				globalVelocity->Accy = -velocity_accel * cos(globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->yaw + abs(localVelocity->theta);
				globalVelocity->Accx = -velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = -velocity_accel * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 4) // x+ y-
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = velocity_accel * sin(globalVelocity->yaw);
				globalVelocity->Accy = -velocity_accel * cos(globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->yaw + localVelocity->theta;
				globalVelocity->Accx = velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = velocity_accel * sin(globalVelocity->yaw);
			}
		}
		break;
	case 2:
		if (quadrant_vel == 1)
		{
			if (localVelocity->Accx == 0) // x+ y+
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = -velocity_accel * cos(globalVelocity->yaw - M_PI / 2);
				globalVelocity->Accy = -velocity_accel * sin(globalVelocity->yaw - M_PI / 2);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标 下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->yaw + localVelocity->theta;
				globalVelocity->Accx = velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = velocity_accel * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 2) // x- y+
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = -velocity_accel * cos(globalVelocity->yaw - M_PI / 2);
				globalVelocity->Accy = -velocity_accel * sin(globalVelocity->yaw - M_PI / 2);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标 下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = M_PI - localVelocity->yaw + abs(localVelocity->theta);
				globalVelocity->Accx = velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = -velocity_accel * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 3) // x- y-
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = velocity_accel * cos(globalVelocity->yaw - M_PI / 2);
				globalVelocity->Accy = velocity_accel * sin(globalVelocity->yaw - M_PI / 2);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标 下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = M_PI - localVelocity->yaw - localVelocity->theta;
				globalVelocity->Accx = velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = -velocity_accel * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 4) // x+ y-
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = velocity_accel * cos(globalVelocity->yaw - M_PI / 2);
				globalVelocity->Accy = velocity_accel * sin(globalVelocity->yaw - M_PI / 2);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标 下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->yaw + localVelocity->theta;
				globalVelocity->Accx = velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = velocity_accel * sin(globalVelocity->yaw);
			}
		}
		break;
	case 3:
		if (quadrant_vel == 1) // x+ y+
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = velocity_accel * sin(globalVelocity->yaw + M_PI);
				globalVelocity->Accy = -velocity_accel * cos(globalVelocity->yaw + M_PI);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标 下的朝向为速度角度朝向+车体朝向
				globalVelocity->yaw = localVelocity->yaw + M_PI + localVelocity->theta;
				globalVelocity->Accx = -velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = -velocity_accel * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 2) // x- y+
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = velocity_accel * sin(globalVelocity->yaw + M_PI);
				globalVelocity->Accy = -velocity_accel * cos(globalVelocity->yaw + M_PI);
			}
			else
			{
				// 计算小车自身big_house合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标
				globalVelocity->yaw = localVelocity->yaw + M_PI + localVelocity->theta;
				globalVelocity->Accx = velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = velocity_accel * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 3) // x- y-
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = velocity_accel * sin(globalVelocity->yaw + M_PI);
				globalVelocity->Accy = -velocity_accel * cos(globalVelocity->yaw + M_PI);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标// x+ y+
				globalVelocity->yaw = localVelocity->yaw + M_PI + localVelocity->theta;
				globalVelocity->Accx = velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = velocity_accel * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 4) // x+ y-
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = -velocity_accel * sin(globalVelocity->yaw + M_PI);
				globalVelocity->Accy = velocity_accel * cos(globalVelocity->yaw + M_PI);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标
				globalVelocity->yaw = localVelocity->yaw + M_PI + localVelocity->theta;
				globalVelocity->Accx = -velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = -velocity_accel * sin(globalVelocity->yaw);
			}
		}
		break;

	case 4:
		if (quadrant_vel == 1) // x+ y+
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = velocity_accel * sin(globalVelocity->yaw + M_PI / 2);
				globalVelocity->Accy = velocity_accel * cos(globalVelocity->yaw + M_PI / 2);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标
				globalVelocity->yaw = -localVelocity->yaw - localVelocity->theta;
				globalVelocity->Accx = velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = -velocity_accel * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 2) // x- y+
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = velocity_accel * sin(-globalVelocity->yaw);
				globalVelocity->Accy = velocity_accel * cos(-globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标
				globalVelocity->yaw = -localVelocity->yaw - localVelocity->theta;
				globalVelocity->Accx = -velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = velocity_accel * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 3) // x- y-
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = -velocity_accel * sin(-globalVelocity->yaw);
				globalVelocity->Accy = -velocity_accel * cos(-globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标
				globalVelocity->yaw = -localVelocity->yaw - localVelocity->theta;
				globalVelocity->Accx = -velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = velocity_accel * sin(globalVelocity->yaw);
			}
		}
		else if (quadrant_vel == 4) // x+ y-
		{
			if (localVelocity->Accx == 0)
			{
				// 此时全局朝向就是小车朝向
				globalVelocity->yaw = localVelocity->yaw;
				globalVelocity->Accx = -velocity_accel * sin(-globalVelocity->yaw);
				globalVelocity->Accy = -velocity_accel * cos(-globalVelocity->yaw);
			}
			else
			{
				// 计算小车自身合成的速度矢量的角度朝向
				localVelocity->theta = atan(localVelocity->Accy / localVelocity->Accx);
				// 全局坐标
				globalVelocity->yaw = -localVelocity->yaw - localVelocity->theta;
				globalVelocity->Accx = velocity_accel * cos(globalVelocity->yaw);
				globalVelocity->Accy = -velocity_accel * sin(globalVelocity->yaw);
			}
		}
		break;
	default:
		break;
	}

	// 根据解算出来的全局速度求出全局角度
	// if(globalVelocity->Accx == 0)
	// {
	// 	if(globalVelocity->Accy < 0)
	// 	{
	// 		globalVelocity->yaw = -M_PI/2;
	// 	}
	// 	else
	// 	{
	// 		globalVelocity->yaw = M_PI/2;
	// 	}
	// }
	// else{
	// 	// 在第一象限
	// 	if(globalVelocity->Accx > 0 && globalVelocity->Accy > 0)
	// 	{
	// 		globalVelocity->yaw = atan(globalVelocity->Accy/globalVelocity->Accx);
	// 	}
	// 	// 在第四象限
	// 	if(globalVelocity->Accx > 0 && globalVelocity->Accy < 0)
	// 	{
	// 		globalVelocity->yaw = atan(globalVelocity->Accy/globalVelocity->Accx);
	// 	} 
	// 	// 在第二象限
	// 	else if(globalVelocity->Accx < 0 && globalVelocity->Accy > 0)
	// 	{
	// 		globalVelocity->yaw = atan(globalVelocity->Accy/globalVelocity->Accx) + M_PI;
	// 	}
	// 	// 在第三象限
	// 	else if(globalVelocity->Accx < 0 && globalVelocity->Accy < 0)
	// 	{
	// 		globalVelocity->yaw = atan(globalVelocity->Accy/globalVelocity->Accx) - M_PI;
	// 	}
	// }
}

}
