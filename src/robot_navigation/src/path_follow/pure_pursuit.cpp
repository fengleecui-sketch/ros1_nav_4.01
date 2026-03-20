/*
 * @Author: your name
 * @Date: 2023-08-22 12:36:47
 * @LastEditTime: 2024-04-19 01:27:26
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/src/path_follow/pure_pursuit.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "path_follow/pure_pursuit.h"

// : private_node("~")用来加载参数
pure_pursuit::pure_pursuit(/* args */) : private_node("~")
{
  // 初始化参数
  Init_Params(private_node);

  // 订阅路径
  pathSub = purePur.subscribe("/opt_path",10,&pure_pursuit::pathCallback,this);

  if(is_use_sim)
  {
    // 订阅真值里程计进行仿真
    localizationSub = purePur.subscribe("/truth_pose_odom", 10, &pure_pursuit::localizationCallback, this);
  }
  else
  {
    localizationSub = purePur.subscribe("/odom_carto", 10, &pure_pursuit::localizationCallback, this);
  }

  // 订阅鼠标点击目标点
  clickSub = purePur.subscribe("/move_base_simple/goal", 10, &pure_pursuit::clickCallback, this);

  // 发布速度
  chassCtlPub = purePur.advertise<geometry_msgs::Twist>("/chassis_control", 1, true);

  // 发布机器人行进方向
  directionPub = purePur.advertise<nav_msgs::Odometry>("/direction",10);

  // 发布全局速度消息
  globalVelPub = purePur.advertise<robot_communication::chassisControl>("/acl_velocity", 1, true);

  ros::Rate LoopRate(control_hz);

  while (ros::ok())
  {
    /* code */
    // 回调函数数据刷新
    ros::spinOnce();
    // 跟随路径
    Pure_Pursuit_Control();
    
    LoopRate.sleep();
  }
  
}

pure_pursuit::~pure_pursuit()
{
}

void pure_pursuit::localizationCallback(const robot_communication::localizationInfoBroadcastConstPtr &msg)
{
  // 获取起点等规划消息
  localData.xPosition = msg->xPosition;
  localData.yPosition = msg->yPosition;
  localData.xSpeed = msg->xSpeed;
  localData.ySpeed = msg->ySpeed;
  localData.xAccel = msg->xAccel;
  localData.yAccel = msg->yAccel;
  localData.chassisGyro = msg->chassisGyro;
  localData.chassisAngle = msg->chassisAngle;

  nowPoint[0] = msg->xPosition;
  nowPoint[1] = msg->yPosition;
  
  // 将底盘的局部坐标系转换一下，才能解算全局世界坐标系
  carVel.Vx = localData.xSpeed;
  carVel.Vy = -localData.ySpeed;
  // 反馈速度合成
  feed_vel_sum = sqrt(pow(fabs(carVel.Vx),2) + pow(fabs(carVel.Vy),2));

  carVel.yaw = localData.chassisAngle;
  vel_transform::LocalVelocityToGlobal(&carVel,&worldVel);

  // 发布里程计消息 用于生成局部代价地图
  directOdom.child_frame_id = "footprint";
  directOdom.header.frame_id = "odom";

  directOdom.pose.pose.position.x = localData.xPosition;
  directOdom.pose.pose.position.y = localData.yPosition;
}

/**
 * @brief:接收鼠标点击 回调函数
 * @note:
 */
void pure_pursuit::clickCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  // 获取规划点
  endPoint[0] = msg->pose.position.x;
  endPoint[1] = msg->pose.position.y;

  // 将路径上打的点清0
  first_yaw_num = 0;
  last_yaw_num = 0;

  // 标志位设定为1
  get_end_flag = true;
}

// 路径回调函数,获取规划出来的路径
void pure_pursuit::pathCallback(const nav_msgs::PathConstPtr &points)
{
  nav_msgs::Path tempath = *points;
  
  if(tempath.poses.size() <= 0)
  {
    return;
  }
  // 获取当前路径点数目
  now_path_nodes_num = tempath.poses.size();

  // 路径更新条件判断，路径长度和终点是否发生变化和终点是否更新
  // 路径长度固定并且没有更新终点
  if(now_path_nodes_num == last_path_nodes_num)
  {
    // 路径没有更新
    update_path = false;
  }
  // 路径长度不相等或终点更新
  if(now_path_nodes_num != last_path_nodes_num)
  {
    // 路径更新
    update_path = true;
  }

  // 清空
  globalPath.clear();
  // 重构大小
  globalPath.resize(tempath.poses.size());
  
  // 路径更新标志位
  // 当更新的时候进行计算
  if(update_path == true)
  {
    // 将路径上打的点清0
    first_yaw_num = 0;
    last_yaw_num = 0;
    // 路径长度归0
    pathlength = 0;
    // x y 方向上的总长度
    length_x_path = 0;
    length_y_path = 0;
    // 传参
    for (int i = 0; i < tempath.poses.size(); i++)
    {
      globalPath[i](0) = tempath.poses[i].pose.position.x;
      globalPath[i](1) = tempath.poses[i].pose.position.y;
      if(i <= tempath.poses.size() - 1)
      {
        // 定义当前点
        Vector2d nowposition = Vector2d(tempath.poses[i].pose.position.x,tempath.poses[i].pose.position.y);
        // 定义下一个点
        Vector2d nextposition = Vector2d(tempath.poses[i+1].pose.position.x,tempath.poses[i+1].pose.position.y);
        // 计算路径总长度
        pathlength += calPointLength(nowposition,nextposition); 

        // 计算x方向上的总长度
        length_x_path += sqrt(pow(abs(nowposition[0]-nextposition[0]),2));
        // 计算y方向上的总长度
        length_y_path += sqrt(pow(abs(nowposition[1]-nextposition[1]),2));
      }
    }
  }

  // 到终点的距离
  double endlength = sqrt(pow(fabs(nowPoint(0)-tempath.poses[tempath.poses.size()-1].pose.position.x),2) +
                  pow(fabs(nowPoint(1)-tempath.poses[tempath.poses.size()-1].pose.position.y),2));
  if(pathlength >  endlength)
  {
    pathlength = endlength;
  }

  has_receive_path = true;

  // 获取上一次的路径
  last_path_nodes_num = now_path_nodes_num;
}

// 计算两个点之间的长度欧氏距离
double pure_pursuit::calPointLength(Vector2d vector1,Vector2d vector2)
{
  return (sqrt((vector1[0]-vector2[0])*(vector1[0]-vector2[0])+(vector1[1]-vector2[1])*(vector1[1]-vector2[1])));
}

void pure_pursuit::Init_Params(ros::NodeHandle &nh)
{
  nh.param("control/car_length",car_length,0.1);  //车的长度
  nh.param("control/time",time,0.1);              //积分时间
  nh.param("control/max_vel",max_velocity,0.5);   //最大速度
  nh.param("control/max_acc",max_acce,0.5);
  nh.param("control/fix_distance",fix_distance,0.05);//前馈距离
  nh.param("control/order",order,3);              // 多项式阶数
  nh.param("control/map_resolution",map_resolution,0.05);   //设定地图分辨率
  nh.param("control/set_vel_k",set_vel_k,1.0);  // 设定路径的系数
  nh.param("control/set_yaw_k",set_yaw_k,1.0);    // 设定yaw的系数
  nh.param("control/hz",control_hz,100);          // 控制频率
  nh.param("control/is_use_sim",is_use_sim,true); // 是否仿真

  // 查看采样点
  sampleNodesPub = nh.advertise<visualization_msgs::Marker>("/Pre_sample",10);

  // pid参数初始化
  pidFollow.Init();
  // 角度跟随PID参数初始化
  pidFollow.SetPIDParameter(&pidFollow.gyro_pid,anglePID);
  // 平滑滤波初始化
  dealdata.Smooth_Filter_Init(20);
  // 二阶低通滤波初始化
  dealdata.LPF2pSetCutoffFreq(1,50,5);
}

void pure_pursuit::Get_CarParams(double vx,double vy,double angle)
{
  // 当前车的速度
  velocity_x = vx;
  velocity_y = vy;
  // 当前车的朝向
  nowangle = angle;
  
  // 速度合成
  feed_vel_sum = sqrt(pow(velocity_x,2)+pow(velocity_y,2));
  feed_vel_sum = min(feed_vel_sum,max_velocity*2);

  // 前搜距离 = vel_sum*t + fro_distance
  pre_distance = feed_vel_sum*time + fro_distance;
}

void pure_pursuit::Deal_Super_Circle(double *setangle,double *feedangle)
{
  /* 
    因为当前的车的临界值是±PI，所以在达到这个角度的时候车只会按照
    正常的累积误差计算
  */
  float delta = *feedangle - *setangle;
  if(delta >= -PI && delta <= PI)
  {
    *feedangle = 0.0f;
    *setangle = delta;
  }
  else if(delta > PI)
  {
    *feedangle = 0.0f;
    *setangle = -PI*2 + delta;
  }
  else if(delta < -PI)
  {
    *feedangle = 0.0f;
    *setangle = PI*2+delta;
  }

  *setangle = -*setangle;
  *feedangle = 0.0f; 
}

void pure_pursuit::planYaw(Vector2d nowpos,vector<Vector2d> path,float &yaw_speed,float &set_yaw)
{
  // 判断路径点 如果路径点<5 不能使用多项式求解
  // 路径点一定要大于多项式的阶数+1
  if(path.size() < order+1)
  {
    return;
  }

  for(int i=0;i<path.size();i++)
  {
    // 当路径点到当前点距离大于向前探索距离的时候记录当前位置点在路径中的位置
    if(fabs((path[i]-nowPoint).norm()) >= pre_distance 
    && fabs((path[i]-nowPoint).norm()) <= pre_distance+0.1)
    {
      if(i>last_yaw_num)
      {
        first_yaw_num = i;
      }
    }
  }
  last_yaw_num = first_yaw_num;

  if(path.size() - first_yaw_num < order+1)
  {
    return;
  }

  // 将路径点转换成矩阵
  MatrixXd xy(order+1,2);
  xy(0,0) = nowPoint[0];
  xy(0,1) = nowPoint[1];

  // 对xy赋值
  for(int i=1;i<xy.rows();i++)
  {
    for(int j=0;j<xy.cols();j++)
    {
      xy(i,j) = path[first_yaw_num+i](j);
    }
  }

  vector<Vector2d> visPrenodes;
  // 查看向前搜索的点
  for (int i = 0; i < order+1; i++)
  {
    // cout<<"当前坐标:"<<path[first_yaw_num+i][0]<<"  "<<path[first_yaw_num+i][1]<<endl;
    visPrenodes.push_back(path[first_yaw_num+i]);
  }

  // 求第1个点的导数，
  double d_k = curve.calDerivative(xy,xy(order,0));
  // 如果d_k接近垂直的时候需要对d_k的正负号进行判断
  double delt_x = xy(order,0) - xy(0,0);
  double delt_y = xy(order,1) - xy(0,1); 
  // 对数据进行处理，进行精度消除，将接近0的数据设定为0
  if(fabs(delt_x) < 0.01)
  {
    delt_x = 0;
  }
  if(fabs(delt_y) < 0.01)
  {
    delt_y = 0;
  }  

  double angle = atan(fabs(d_k));

  // 设定角度
  if(delt_x == 0 && delt_y > 0)
  {
    angle = M_PI_2;
  }
  else if(delt_x == 0 && delt_y < 0)
  {
    angle = -M_PI_2;
  }
  else if(delt_x > 0 && delt_y == 0)
  {
    angle = 0;
  }
  else if(delt_x < 0 && delt_y == 0)
  {
    angle = -M_PI;
  }
  else if(delt_x > 0 && delt_y > 0)
  {
    angle = atan(fabs(d_k));
  }
  else if(delt_x > 0 && delt_y < 0)
  {
    angle = -atan(fabs(d_k));
  }
  else if(delt_x < 0 && delt_y < 0)
  {
    angle = atan(fabs(d_k)) - M_PI;
  }
  else if(delt_x < 0 && delt_y > 0)
  {
    angle = M_PI - atan(fabs(d_k));
  }

  // 平滑滤波输出角度 二阶低通滤波输出角度
  double outputangle = 0.0,filter_angle = 0.0;
  // 平滑滤波
  dealdata.Smooth_Filter(angle,outputangle);
  // 二阶地同滤波算法
  filter_angle = dealdata.LPF2pApply(1,outputangle);
  setangle = filter_angle;
  if(fabs(setangle-lastangle) <= 0.05)
  {
    setangle = lastangle;
  }
  lastangle = filter_angle;
  

  // 由于和车的坐标系不完全一致，现在需要进行简单的转换
  double setcarangle = setangle-M_PI/2.0f;

  // 将角度限制在-pi到pi之间
  if(setcarangle > M_PI)
  {
    setcarangle -= 2*M_PI;
  }
  else if(setcarangle < -M_PI)
  {
    setcarangle += 2*M_PI;
  }  

  // 真实场景下的
  if(!is_use_sim)
  {    
    // 计算角速度
    pidFollow.gyro_pid.SetPoint = setcarangle;
    pidFollow.gyro_pid.feedPoint = localData.chassisAngle;
    Deal_Super_Circle(&pidFollow.gyro_pid.SetPoint,&pidFollow.gyro_pid.feedPoint);
    pidFollow.PIDFloatPositionCal(&pidFollow.gyro_pid);

    // 限制幅度
    pidFollow.gyro_pid.OutPoint = LIMIT(pidFollow.gyro_pid.OutPoint,-2*M_PI,2*M_PI);

    // 底盘需要控制的角度
    set_yaw = setcarangle;
    // 计算Yaw轴角度
    yaw_speed = pidFollow.gyro_pid.OutPoint;
  }
  // 仿真
  else
  {
    // 需要把世界坐标系下的速度进行一下简单的处理转换为当前坐标系
    double local_vel_x;
    double local_vel_y;
    // 车体局部速度和全局速度
    Odom_data_define localVel;
    Odom_data_define globalVel;

    globalVel.Vx = setVeloc_x;
    globalVel.Vy = setVeloc_y;

    // 获取当前yaw角度
    localVel.yaw = localData.chassisAngle;

    vel_transform::GlobalVelocityToLocal(&localVel,&globalVel);

    local_vel_x = localVel.Vx;
    local_vel_y = localVel.Vy;

    pidFollow.gyro_pid.SetPoint = setangle;
    pidFollow.gyro_pid.feedPoint = localData.chassisAngle;
    Deal_Super_Circle(&pidFollow.gyro_pid.SetPoint,&pidFollow.gyro_pid.feedPoint);
    pidFollow.PIDFloatPositionCal(&pidFollow.gyro_pid);

    // 底盘需要控制的角度
    set_yaw = setcarangle;
    yaw_speed = pidFollow.gyro_pid.OutPoint;
  }  
}

// 规划速度
// 当前点 nowpos
// 路径 path
// 输出x轴方向速度 x_velocity
// 输出y轴方向速度 y_velocity
void pure_pursuit::planVelocity(Vector2d nowpos,vector<Vector2d> path,float &x_velocity,float &y_velocity)
{
  // 判断路径点 如果路径点<5 不能使用多项式求解
  // 路径点一定要大于多项式的阶数+1
  if(path.size() < order+1)
  {
    Vector2d controlVel;
    controlVel = pidFollow.speedOutput(nowpos,path.back(),path);
    x_velocity = controlVel(0);
    y_velocity = controlVel(1);

    return;
  }

  int point_num;
  Vector2d localpoint = caLocalGoalPosition(path,nowpos,pre_distance,point_num);
  
  setVeloc_x = localpoint[0]-nowPoint[0];
  setVeloc_y = localpoint[1]-nowPoint[1];

  double distance = calPointLength(nowPoint,path[path.size()-1]);
  if(point_num == path.size()-1 && distance <= 0.2)
  {
    setVeloc_x = 0;
    setVeloc_y = 0;
  }

  vector<Vector2d> tempvec;
  tempvec.push_back(localpoint);
  visual_VisitedNode(sampleNodesPub,tempvec,0.5,0,1,0.5,5);

  // 真实场景下的
  if(!is_use_sim)
  {
    // 对速度进行限制
    LIMIT_VECTOR_Velocity(setVeloc_x,setVeloc_y,max_velocity);
    x_velocity = setVeloc_y;
    y_velocity = -setVeloc_x;       
  }
  // 仿真
  else
  {
    // 需要把世界坐标系下的速度进行一下简单的处理转换为当前坐标系
    double local_vel_x;
    double local_vel_y;
    // 车体局部速度和全局速度
    Odom_data_define localVel;
    Odom_data_define globalVel;

    globalVel.Vx = setVeloc_x;
    globalVel.Vy = setVeloc_y;

    // 获取当前yaw角度
    localVel.yaw = localData.chassisAngle;

    vel_transform::GlobalVelocityToLocal(&localVel,&globalVel);

    local_vel_x = localVel.Vx;
    local_vel_y = localVel.Vy;

    // 对速度进行限制
    LIMIT_VECTOR_Velocity(local_vel_x,local_vel_y,max_velocity);
    x_velocity = local_vel_x;
    y_velocity = local_vel_y;      
  }
}

// 用来对速度进行限制,因为机器人是全向移动的,如果直接通过数值对x y速度限制会产生畸变
// 所以需要先合成,再进行限制
// vel_x 输入的x方向上的速度
// vel_y 输入的y方向上的速度
// limit_velocity 限制的速度
void pure_pursuit::LIMIT_VECTOR_Velocity(double &vel_x,double &vel_y,double limit_velocity)
{
  // 获取速度
  double temp_x_velocity = abs(vel_x);
  double temp_y_velocity = abs(vel_y);
  // 总的速度
  double sum_velocity = sqrt(pow(temp_x_velocity,2)+pow(temp_y_velocity,2));

  // 判断速度大小
  if(sum_velocity > limit_velocity)
  {
    // 求解缩小比例
    double scale_down = sum_velocity/limit_velocity;
    // 求解速度
    vel_x = vel_x/scale_down;
    vel_y = vel_y/scale_down;
  }
  else
  {
    vel_x = vel_x;
    vel_y = vel_y;
  }
}

// 用来求解当前位置点以一定半径相交的路径点
// path 输入路径
// radius 搜索半径
// point_num 返回当前点
// return 局部终点
Vector2d pure_pursuit::caLocalGoalPosition(vector<Vector2d> path,Vector2d nowpoint,double radius,int &point_num)
{
  Vector2d localgoal;
  int path_length_num = path.size();

  // 计算当前位置到路径点上的最小距离
  double min_distance = 1000;
  int now_num;    //定位当前点再路径上的位置
  for (int i = 0; i < path_length_num; i++)
  {
    // 计算距离
    double distance = calPointLength(path[i],nowpoint);
    if(distance < min_distance)
    {
      min_distance = distance;
      now_num = i;
    }
  }

  int traj_num;
  traj_num = now_num+1;
  for (int i = now_num+1; i < path_length_num; i++)
  {
    // 计算距离
    double distance = calPointLength(path[now_num],path[i]);
    if(distance >= radius && distance < radius + 0.1)
    {
      traj_num = i;
      break;
    }
  }

  // 确定下一个点的采样位置
  if(traj_num < path_length_num-1)
  {
    localgoal = path[traj_num];
    point_num = traj_num;
  }
  else
  {
    localgoal = path[path_length_num-1];
    point_num = path_length_num-1;
  }

  return localgoal;
}

// 动态五次多项式规划速度
// 当前点 nowpos
// 终点 endpos
// 当前速度 nowvel
// 终点速度 endvel
// 当前点加速度 nowacc
// 终点加速度   endacc
// 路径 nowpath
void pure_pursuit::Dynamic_Quintic_Polynomial(Vector2d nowpos,Vector2d endpos,
Vector2d nowvel,Vector2d endvel,
Vector2d nowacc,Vector2d endacc,vector<Vector2d> nowpath)
{
  // 两个时间点，当前时刻设定为0，下个时刻为0.02
  double t0 = 0,t1 = 1.0f/control_hz;
  // 构建x方向上的五次多项式
  VectorXd x_direct(6);
  // 第一个元素是当前的x位置
  x_direct[0] = nowpos[0];
  // 第二个元素是当前的x速度
  x_direct[1] = nowvel[0];
  // 第三个元素是当前的x加速度
  x_direct[2] = nowacc[0];

    // 下一个点的参数需要根据当前点在路径中的位置进行确定
    // 第四个是下一个点的x位置
    x_direct[3] = endpos[0];
    // 第五个是下一个点的x速度
    x_direct[4] = endvel[0];
    // 第六个是下一个点的x加速度
    x_direct[5] = endacc[0];

  // 构建y方向上的五次多项式
  VectorXd y_direct(6);
  // 第一个元素是当前的y位置
  y_direct[0] = nowpos[1];
  // 第二个元素是当前的x速度
  y_direct[0] = nowvel[1];
  // 第三个元素是当前的x加速度
  y_direct[0] = nowacc[1];
  // 下一个点的参数需要根据当前点在路径中的位置进行确定
    // 第四个是下一个点的x位置
    y_direct[3] = endpos[1];
    // 第五个是下一个点的x速度
    y_direct[4] = endvel[1];
    // 第六个是下一个点的x加速度
    y_direct[5] = endacc[1];

  // 计算当前点到终点的距离长度，分成x和y两部分进行计算，得出当前的t

  // 计算占比

  // 根据宏观的五次多项式求解当前的t

  // 计算出来当前的t,求解下一个点的t解算速度
}

// 五次多项式路径跟随
// nowpos  当前位置点
// path  路径
void pure_pursuit::Pure_Pursuit_Control(void)
{
  if(!has_receive_path)
  {
    return;
  }
  
  get_end_flag = false;

  yawtrajPath.clear();
  yawtrajPath = globalPath;
  veltrajPath.clear();
  veltrajPath = globalPath;

  // 规划yaw
  planYaw(nowPoint,yawtrajPath,motionData.chassisGyro,motionData.chassisAngle);

  motionData.chassisGyro = 0;
  motionData.chassisAngle = 0;

  // 设定向前探索距离 pre_d = fix_d + (1/control_hz)*vel
  pre_distance = fix_distance + (1.0/control_hz)*feed_vel_sum;
  
  // 当反馈有速度的时候时间进行累加
  if(feed_vel_sum >= 0.2)
  {
    integral_t = (1.0/control_hz);
  }
  else
  {
    integral_t = 0.0;
  }

  // 规划速度
  planVelocity(nowPoint,veltrajPath,motionData.xSpeed,motionData.ySpeed);
  
  // 把仿真和实车区分开
  // 实车
  if(!is_use_sim)
  {
    //  
    globalVelPub.publish(motionData);
  }
  // 仿真 
  else
  {
    vel_control.linear.x = motionData.xSpeed;
    vel_control.linear.y = motionData.ySpeed;
    vel_control.angular.z = motionData.chassisGyro;
    
    cout<<"xSpeed:"<<motionData.xSpeed<<endl;
    cout<<"ySpeed:"<<motionData.ySpeed<<endl;
    // cout<<"set angle"<<motionData.chassisAngle<<endl;
    cout<<"chassisGyro:"<<motionData.chassisGyro<<endl;

    chassCtlPub.publish(vel_control);
  }

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = setangle;

  // 在rviz中显示当前规划的方向
  // 转换为 geometry_msgs::Quaternion
  geometry_msgs::Quaternion quaternion_msg;
  quaternion_msg = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);

  directOdom.pose.pose.orientation.x = quaternion_msg.x;
  directOdom.pose.pose.orientation.y = quaternion_msg.y;
  directOdom.pose.pose.orientation.z = quaternion_msg.z;
  directOdom.pose.pose.orientation.w = quaternion_msg.w;

  directionPub.publish(directOdom);
}

// nowpos 当前位置点
// path 路径
// 初步拟定使用6个点，即5次多项式进行轨迹跟随，输出的是速度
void pure_pursuit::control(Vector2d nowpos,vector<Vector2d> path)
{
  if(!has_receive_path)
  {
    return;
  }

  // 判断路径点 如果路径点<5 不能使用多项式求解
  // 路径点一定要大于多项式的阶数+1
  if(path.size() < order+1)
  {
    return;
  }

  // 路径长度小于0.1停止规划
  if(pathlength < 0.1)
  {
    return;
  }

  int num = 0;
  for(int i=0;i<path.size();i++)
  {
    if(fabs((path[i]-nowPoint).norm()) >= 0.1)
    {
      num = i;
      break;
    }
  }

  if(path.size() - num < order+1)
  {
    return;
  }

  // 将路径点转换成矩阵
  MatrixXd xy(order+1,2);

  // 对xy赋值
  for(int i=0;i<xy.rows();i++)
  {
    for(int j=0;j<xy.cols();j++)
    {
      xy(i,j) = path[num+i](j);
    }
  }

  double vel_sum = 1.0;

  // 求第1个点的导数，
  double d_k = curve.calDerivative(xy,xy(order,0));

  // 如果d_k接近垂直的时候需要对d_k的正负号进行判断
  double delt_x = xy(order,0) - xy(0,0);
  double delt_y = xy(order,1) - xy(0,1); 
  if(fabs(delt_x) < 0.01)
  {
    delt_x = 0;
  }
  if(fabs(delt_y) < 0.01)
  {
    delt_y = 0;
  }  

  double angle = atan(fabs(d_k));

  // 设定速度
  if(delt_x < 0)
  {
    setVeloc_x = -1*vel_sum*cos(angle);
  }
  else if(delt_x > 0)
  {
    setVeloc_x = 1*vel_sum*cos(angle);
  }

  if(delt_y < 0)
  {
    setVeloc_y = -1*vel_sum*sin(angle);
  }
  else if(delt_y > 0)
  {
    setVeloc_y = 1*vel_sum*sin(angle);
  }

  if(delt_x == 0)
  {
    setVeloc_x = 0;
  }
  if(delt_y == 0)
  {
    setVeloc_y = 0;
  }

  // 设定角度
  if(delt_x == 0 && delt_y > 0)
  {
    angle = M_PI_2;
  }
  else if(delt_x == 0 && delt_y < 0)
  {
    angle = -M_PI_2;
  }
  else if(delt_x > 0 && delt_y == 0)
  {
    angle = 0;
  }
  else if(delt_x < 0 && delt_y == 0)
  {
    angle = -M_PI;
  }
  else if(delt_x > 0 && delt_y > 0)
  {
    angle = atan(fabs(d_k));
  }
  else if(delt_x > 0 && delt_y < 0)
  {
    angle = -atan(fabs(d_k));
  }
  else if(delt_x < 0 && delt_y < 0)
  {
    angle = atan(fabs(d_k)) - M_PI;
  }
  else if(delt_x < 0 && delt_y > 0)
  {
    angle = M_PI - atan(fabs(d_k));
  }
  
  double outputangle = 0.0,filter_angle = 0.0;
  dealdata.Smooth_Filter(angle,outputangle);
  filter_angle = dealdata.LPF2pApply(1,outputangle);
  setangle = filter_angle;
  if(fabs(setangle-lastangle) <= 0.05)
  {
    setangle = lastangle;
  }
  lastangle = filter_angle;
  

  // 由于和车的坐标系不完全一致，现在需要进行简单的转换
  double setcarangle = setangle-M_PI/2.0f;

  // 将角度限制在-pi到pi之间
  if(setcarangle > M_PI)
  {
    setcarangle -= 2*M_PI;
  }
  else if(setcarangle < -M_PI)
  {
    setcarangle += 2*M_PI;
  }

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = setangle;

  // 转换为 geometry_msgs::Quaternion
  geometry_msgs::Quaternion quaternion_msg;
  quaternion_msg = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);

  directOdom.pose.pose.orientation.x = quaternion_msg.x;
  directOdom.pose.pose.orientation.y = quaternion_msg.y;
  directOdom.pose.pose.orientation.z = quaternion_msg.z;
  directOdom.pose.pose.orientation.w = quaternion_msg.w;

  directionPub.publish(directOdom);

  // 真实场景下的
  if(!is_use_sim)
  {    
    // 计算角速度
    pidFollow.gyro_pid.SetPoint = setcarangle;
    pidFollow.gyro_pid.feedPoint = localData.chassisAngle;
    Deal_Super_Circle(&pidFollow.gyro_pid.SetPoint,&pidFollow.gyro_pid.feedPoint);
    pidFollow.PIDFloatPositionCal(&pidFollow.gyro_pid);

    // 限制幅度
    pidFollow.gyro_pid.OutPoint = LIMIT(pidFollow.gyro_pid.OutPoint,-2*M_PI,2*M_PI);

    // 底盘需要控制的角度
    motionData.chassisAngle = setcarangle;
    motionData.chassisGyro = pidFollow.gyro_pid.OutPoint;
    // motionData.chassisGyro = 0;
    // 相对于底盘的世界坐标系下的速度
    // 因为底盘的世界坐标系和规划的全局不一样，需要进行变换
    motionData.xSpeed = setVeloc_y;
    motionData.ySpeed = -setVeloc_x;

    globalVelPub.publish(motionData);
  }
  // 仿真
  else
  {
    // 需要把世界坐标系下的速度进行一下简单的处理转换为当前坐标系
    double local_vel_x;
    double local_vel_y;
    // 车体局部速度和全局速度
    Odom_data_define localVel;
    Odom_data_define globalVel;

    globalVel.Vx = setVeloc_x;
    globalVel.Vy = setVeloc_y;

    // 获取当前yaw角度
    localVel.yaw = localData.chassisAngle;

    vel_transform::GlobalVelocityToLocal(&localVel,&globalVel);

    local_vel_x = localVel.Vx;
    local_vel_y = localVel.Vy;

    pidFollow.gyro_pid.SetPoint = setangle;
    pidFollow.gyro_pid.feedPoint = localData.chassisAngle;
    Deal_Super_Circle(&pidFollow.gyro_pid.SetPoint,&pidFollow.gyro_pid.feedPoint);
    pidFollow.PIDFloatPositionCal(&pidFollow.gyro_pid);

    vel_control.linear.x = local_vel_x;
    vel_control.linear.y = local_vel_y;
    vel_control.angular.z = pidFollow.gyro_pid.OutPoint;

    chassCtlPub.publish(vel_control);
  }
}

void pure_pursuit::visual_VisitedNode(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> visitnodes,
float a_set,float r_set,float g_set,float b_set,float length)
{
  visualization_msgs::Marker node_vis;
  node_vis.header.frame_id = "map";
  node_vis.header.stamp = ros::Time::now();

  node_vis.color.a = a_set;
  node_vis.color.r = r_set;
  node_vis.color.g = g_set;
  node_vis.color.b = b_set;
  node_vis.ns = "fast_security_visited";

  node_vis.type = visualization_msgs::Marker::CUBE_LIST;
  node_vis.action = visualization_msgs::Marker::ADD;
  node_vis.id = 0;

  node_vis.pose.orientation.x = 0.0;
  node_vis.pose.orientation.y = 0.0;
  node_vis.pose.orientation.z = 0.0;
  node_vis.pose.orientation.w = 1.0;

  node_vis.scale.x = 0.05*length;
  node_vis.scale.y = 0.05*length;
  node_vis.scale.z = 0.05*length;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(visitnodes.size()); i++)
  {
    pt.x = visitnodes[i][0];
    pt.y = visitnodes[i][1];
    node_vis.points.push_back(pt);
  }

  pathPublish.publish(node_vis);
}

