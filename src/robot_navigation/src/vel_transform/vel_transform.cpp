#include "vel_transform/vel_transform.h"

namespace vel_transform
{
// 计算单位向量
Vector2d calUnitvector(Vector2d unitv)
{
  // 计算单位向量
  unitv = unitv * 1.0f/(sqrt(pow(abs(unitv[0]),2)+pow(abs(unitv[1]),2)));
  return unitv;
}

// 计算向量之间的夹角
double calVectorAngle(Vector2d vector1,Vector2d vector2)
{
  // 先单位化
  Vector2d vectorFirst = calUnitvector(vector1);
  // 
  Vector2d vectorSecond = calUnitvector(vector2);

  // 向量乘积
  double vector_angle = vectorFirst[0]*vectorSecond[0] + vectorFirst[1]*vectorSecond[1];
  // 计算夹角
  return acos(vector_angle);
}

// 计算两个点之间的长度欧氏距离
double calPointLength(Vector2d vector1,Vector2d vector2)
{
  return (sqrt((vector1[0]-vector2[0])*(vector1[0]-vector2[0])+(vector1[1]-vector2[1])*(vector1[1]-vector2[1])));
}


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
	int quadrant_yaw = 0; // 定义车体所在象限 1-4
	int quadrant_vel = 0; // 车的局部速度合成
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
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
			cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
			cout<<"quadrant_vel:"<<quadrant_vel<<endl;
		}
		break;
	
	default:
		break;
	}
}

// 全局速度转局部
void GlobalVelocityToLocalVelocity(Odom_data_define *localVelocity, Odom_data_define *globalVelocity)
{
	// 数值过小直接忽略
	// if (abs(globalVelocity->Vx) < 0.2)
	// {
	// 	globalVelocity->Vx = 0;
	// }
	// if (abs(globalVelocity->Vy) < 0.2)
	// {
	// 	globalVelocity->Vy = 0;
	// }	
	
	// 全局速度矢量
	Vector2d velocity_vec = Vector2d(globalVelocity->Vx,globalVelocity->Vy);
	// 水平向量即正方向朝向
	Vector2d horizontal_vec = Vector2d(1.0,0.0);
	// 计算夹角 这里返回的夹角是弧度制的，范围是0-pi
	double velocity_angle = 0;
	if(globalVelocity->Vx != 0 || globalVelocity->Vy != 0)
	{
		velocity_angle = calVectorAngle(velocity_vec,horizontal_vec);
	}

	// 因为我们当前返回的车的角度是带有坐标的，并且范围是-pi---+pi
	// 所以我们需要把全局速度转换到这个范围
	// 第四象限 对计算出来的角度-pi
	if(velocity_vec[0] > 0 && velocity_vec[1] < 0)
	{
		velocity_angle = -velocity_angle;
	}
	// 第三象限 对计算出来的角度-pi
	if(velocity_vec[0] < 0 && velocity_vec[1] < 0)
	{
		velocity_angle = -velocity_angle;
	}
	// 当x = 0 y<0的时候
	if(velocity_vec[0] == 0 && velocity_vec[1] < 0)
	{
		velocity_angle = -M_PI/2;
	}

	// 速度解算
	double velocity_sum;		// 合成速度
	int quadrant_yaw = 0; // 定义车体所在象限 1-4
	int quadrant_vel = 0; // 车的局部速度合成
	velocity_sum = sqrt(pow(globalVelocity->Vx, 2) + pow(globalVelocity->Vy, 2));

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
	if (globalVelocity->Vx>= 0 && globalVelocity->Vy>= 0)
	{
		quadrant_vel = 1;
	}
	// x- y+
	else if (globalVelocity->Vx < 0 && globalVelocity->Vy> 0)
	{
		quadrant_vel = 2;
	}
	// x- y-
	else if (globalVelocity->Vx < 0 && globalVelocity->Vy < 0)
	{
		quadrant_vel = 3;
	}
	// x+ y-
	else if (globalVelocity->Vx > 0 && globalVelocity->Vy < 0)
	{
		quadrant_vel = 4;
	}

	double theta;
	// 判断车的姿态在第几象限
	switch (quadrant_yaw)
	{
	case 1:	//第一象限
		if(quadrant_vel == 1)	// 全局速度 x+ y+
		{
			// 合成车底盘的速度矢量角度
			theta = velocity_angle-localVelocity->yaw;
			// 解算局部速度
			localVelocity->Vx = velocity_sum*cos(theta);
			localVelocity->Vy = velocity_sum*sin(theta);
		}
		else if(quadrant_vel == 2) //全局速度 x- y+
		{
			// 合成车底盘的速度矢量角度
			theta = velocity_angle - M_PI_2 - localVelocity->yaw;
			// 解算局部速度
			localVelocity->Vx = -velocity_sum*sin(theta);
			localVelocity->Vy = velocity_sum*cos(theta);
		}
		else if(quadrant_vel == 3)	//全局速度 x- y-
		{
			// 合成车底盘的速度矢量角度
			theta= -velocity_angle - M_PI_2 - localVelocity->yaw;
			// 解算局部速度
			localVelocity->Vx = -velocity_sum*cos(theta);
			localVelocity->Vy = -velocity_sum*sin(theta);
		}
		else if(quadrant_vel == 4)	//全局速度 x+ y-
		{
			// 合成车底盘的速度矢量角度
			theta = -velocity_angle + localVelocity->yaw;
			// 解算局部速度
			localVelocity->Vx = velocity_sum*cos(theta);
			localVelocity->Vy = -velocity_sum*sin(theta);
		}
		break;
	case 2:		//第二象限
		if(quadrant_vel == 1)		//全局速度x+ y+
		{
			// 合成车底盘的速度矢量角度
			theta = localVelocity->yaw-velocity_angle-M_PI_2;
			//解算局部速度
			localVelocity->Vx = -velocity_sum*sin(theta);
			localVelocity->Vy = -velocity_sum*cos(theta);
		}
		else if(quadrant_vel == 2)	//全局速度x- y+
		{
			// 合成车底盘的速度矢量角度
			theta = localVelocity->yaw - velocity_angle;
			// 解算局部速度
			localVelocity->Vx = velocity_sum*cos(theta);
			localVelocity->Vy = -velocity_sum*sin(theta);
		}
		else if(quadrant_vel == 3)	//全局速度x- y-
		{
			// 合成车底盘的速度矢量角度
			theta = velocity_angle - localVelocity->yaw;
			// 解算局部速度
			localVelocity->Vx = velocity_sum*cos(theta);
			localVelocity->Vy = velocity_sum*sin(theta);
		}
		else if(quadrant_vel == 4)	//全局速度 x+ y-
		{
			// 合成车底盘的速度矢量角度
			theta = -velocity_angle-M_PI+localVelocity->yaw;
			// 解算局部速度
			localVelocity->Vx = -velocity_sum*cos(theta);
			localVelocity->Vy = velocity_sum*sin(theta);
		}
		break;
	case 3:	//第三象限
		if(quadrant_vel == 1)	// 全局速度 x+ y+
		{
			// 合成车底盘的速度矢量角度
			theta = velocity_angle - M_PI - localVelocity->yaw;
			// 解算局部速度
			localVelocity->Vx = -velocity_sum*cos(theta);
			localVelocity->Vy = -velocity_sum*sin(theta);
		}
		else if(quadrant_vel == 2) //全局速度 x- y+
		{
			// 合成车底盘的速度矢量角度
			theta = M_PI-velocity_angle+M_PI+localVelocity->yaw;
			// 解算局部速度
			localVelocity->Vx = velocity_sum*cos(theta);
			localVelocity->Vy = -velocity_sum*sin(theta);
		}
		else if(quadrant_vel == 3)	//全局速度 x- y-
		{
			// 合成车底盘的速度矢量角度
			theta= M_PI + velocity_angle - M_PI - localVelocity->yaw;
			// 解算局部速度
			localVelocity->Vx = velocity_sum*cos(theta);
			localVelocity->Vy = velocity_sum*sin(theta);
		}
		else if(quadrant_vel == 4)	//全局速度 x+ y-
		{
			// 合成车底盘的速度矢量角度
			theta = -M_PI_2 - localVelocity->yaw + velocity_angle;
			// 解算局部速度
			localVelocity->Vx = -velocity_sum*sin(theta);
			localVelocity->Vy = velocity_sum*cos(theta);
		}
		break;
	case 4:	//第四象限
		if(quadrant_vel == 1)
		{
			// 合成车底盘的速度矢量角度
			theta = velocity_angle - localVelocity->yaw;
			// 解算局部速度
			localVelocity->Vx = velocity_sum*cos(theta);
			localVelocity->Vy = velocity_sum*sin(theta);
		}
		else if(quadrant_vel == 2)
		{
			// 合成车底盘的速度矢量角度
			theta = velocity_angle - M_PI_2 - localVelocity->yaw;
			// 解算局部速度
			localVelocity->Vx = -velocity_sum*sin(theta);
			localVelocity->Vy = velocity_sum*cos(theta);
		}
		else if(quadrant_vel == 3)
		{
			// 合成车底盘的速度矢量角度
			theta = M_PI + velocity_angle - localVelocity->yaw;
			// 解算局部速度
			localVelocity->Vx = -velocity_sum*cos(theta);
			localVelocity->Vy = -velocity_sum*sin(theta);
		}
		else if(quadrant_vel == 4)
		{
			// 合成车底盘的速度矢量角度
			theta = -velocity_angle+localVelocity->yaw;
			// 解算局部速度
			localVelocity->Vx = velocity_sum*cos(theta);
			localVelocity->Vy = -velocity_sum*sin(theta);
		}
		break;
	
	default:
		break;
	}

	// cout<<"yaw angle:"<<localVelocity->yaw<<endl;
	// cout<<"quadrant_yaw:"<<quadrant_yaw<<endl;
	// cout<<"quadrant_vel:"<<quadrant_vel<<endl;
	// cout<<"global velocity x:"<<globalVelocity->Vx<<endl;
	// cout<<"global velocity y:"<<globalVelocity->Vy<<endl;
	// cout<<"local velocity x:"<<localVelocity->Vx<<endl;
	// cout<<"local velocity y:"<<localVelocity->Vy<<endl;
}

// 采用向量方法将全局速度转成局部速度
void GlobalVelocityToLocalVector(Odom_data_define *localVelocity, Odom_data_define *globalVelocity)
{
	// 水平向量即正方向朝向
	Vector2d horizontal_vec = Vector2d(1.0,0.0);
	// 获取当前的全局速度向量
	Vector2d global_vel_vec = Vector2d(globalVelocity->Vx,globalVelocity->Vy);
	// 全局速度合成
	double global_sum = sqrt(pow(globalVelocity->Vx, 2) + pow(globalVelocity->Vy, 2));
	// 计算全局速度朝向
	if (globalVelocity->Vx != 0 || globalVelocity->Vy != 0 )
	{
		/* code */
		globalVelocity->yaw = calVectorAngle(horizontal_vec,global_vel_vec);
	}
	else
	{
		globalVelocity->yaw = 0;
	}
	
	// 如果globalVelocity->Vy > 0 则global_yaw > 0 若小于0 则global_yaw < 0
	if (globalVelocity->Vy < 0)
	{
		globalVelocity->yaw = -globalVelocity->yaw;
	}
	// 判断全局速度朝向和车的角度是否在同一半圆范围内，即符号是否一样
	if (globalVelocity->yaw * localVelocity->yaw >= 0)
	{
		/* code */
		double delt_yaw = globalVelocity->yaw - localVelocity->yaw;
		localVelocity->Vx = global_sum*cos(delt_yaw);
		localVelocity->Vy = global_sum*sin(delt_yaw);
	}
	if (globalVelocity->yaw * localVelocity->yaw < 0)
	{
		// 如果车的姿态角度<0
		if (localVelocity->yaw < 0)
		{
			// 计算角度差
			double delt_yaw = globalVelocity->yaw - (M_PI+localVelocity->yaw);
			localVelocity->Vx = -global_sum*cos(delt_yaw);
			localVelocity->Vy = -global_sum*sin(delt_yaw);
		}
		// 如果车的姿态角度<0
		if (localVelocity->yaw > 0)
		{
			// 计算角度差
			double delt_yaw = globalVelocity->yaw - (localVelocity->yaw-M_PI);
			localVelocity->Vx = -global_sum*cos(delt_yaw);
			localVelocity->Vy = -global_sum*sin(delt_yaw);
		}
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
