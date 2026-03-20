#include "local_plan/omnidirectional_dwa.h"

Omnidirectional_DWAPlanner::Omnidirectional_DWAPlanner(void)
    : local_nh("~"), local_goal_subscribed(false), scan_updated(false), local_map_updated(false), odom_updated(false)
{
  local_nh.param("HZ", HZ, {20});
  local_nh.param("IS_USE_ESDF", IS_USE_ESDF, true);
  local_nh.param("IS_SIM", IS_SIM, true);
  local_nh.param("TEMP_GOAL_RADIUS", TEMP_GOAL_RADIUS, {20});
  local_nh.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
  local_nh.param("MAX_VELOCITY_X", MAX_VELOCITY_X, {1.0});
  local_nh.param("MIN_VELOCITY_X", MIN_VELOCITY_X, {0.0});
  local_nh.param("MAX_VELOCITY_Y", MAX_VELOCITY_Y, {1.0});
  local_nh.param("MIN_VELOCITY_Y", MIN_VELOCITY_Y, {0.0});
  local_nh.param("MAX_YAWRATE", MAX_YAWRATE, {0.8});
  local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
  local_nh.param("MAX_D_YAWRATE", MAX_D_YAWRATE, {2.0});
  local_nh.param("MAX_DIST", MAX_DIST, {10.0});
  local_nh.param("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION, {0.1});
  local_nh.param("YAWRATE_RESOLUTION", YAWRATE_RESOLUTION, {0.1});
  local_nh.param("ANGLE_RESOLUTION", ANGLE_RESOLUTION, {0.2});
  local_nh.param("PREDICT_TIME", PREDICT_TIME, {3.0});
  local_nh.param("TO_GOAL_COST_GAIN", TO_GOAL_COST_GAIN, {1.0});
  local_nh.param("SPEED_COST_GAIN", SPEED_COST_GAIN, {1.0});
  local_nh.param("OBSTACLE_COST_GAIN", OBSTACLE_COST_GAIN, {1.0});
  local_nh.param("GOAL_THRESHOLD", GOAL_THRESHOLD, {0.3});
  local_nh.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {1.0});
  DT = 1.0 / HZ;
  
  // 跟随路径集合的发布
  candidate_trajectories_pub = nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
  // 选定的跟随路径
  selected_trajectory_pub = nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
  // 发布局部规划路径
  localPathPub = nh.advertise<nav_msgs::Path>("/local_path",1);

  // 如果是仿真
  if(IS_SIM)
  {
    // 发布局部规划出来的速度
    chassCtlPub = nh.advertise<geometry_msgs::Twist>("/chassis_control", 1, true);
  }
  // 如果不是仿真
  else
  {
    // 发布实际的速度
    chassCtlPub = nh.advertise<robot_communication::chassisControl>("/acl_velocity", 1, true);
  }

  // 查看局部终点
  localgoalPub = nh.advertise<visualization_msgs::Marker>("/local_goal",10);

  // 订阅鼠标点击目标点
  clickSub = nh.subscribe("/move_base_simple/goal", 10, &Omnidirectional_DWAPlanner::clickCallback, this);
  // clickSub = nh.subscribe("/globalEnd", 10, &Omnidirectional_DWAPlanner::clickCallback, this);
    // 订阅路径
  pathSub = nh.subscribe("/opt_path",10,&Omnidirectional_DWAPlanner::pathCallback,this);

  local_map_sub = nh.subscribe("/local_map_esdf", 1, &Omnidirectional_DWAPlanner::local_map_callback, this);

  // 仿真和实际的定位消息是不一样的
  if (IS_SIM == false)
  {
    odom_sub = nh.subscribe("/odom_carto", 1, &Omnidirectional_DWAPlanner::odom_callback, this);
  }
  else
  {
    odom_sub = nh.subscribe("/truth_pose_odom", 1, &Omnidirectional_DWAPlanner::odom_callback, this);
  }

  // pid参数初始化
  pidFollow.Init();
  // 角度跟随PID参数初始化
  pidFollow.SetPIDParameter(&pidFollow.gyro_pid,anglePID);
}

// 状态参数初始化
Omnidirectional_DWAPlanner::State::State(double _x, double _y, double _yaw, double _velocity_x ,double _velocity_y, double _yawrate)
    : x(_x), y(_y), yaw(_yaw), velocity_x(_velocity_x),velocity_y(_velocity_y) ,yawrate(_yawrate)
{
}

Omnidirectional_DWAPlanner::Window::Window(void)
    : min_velocity_x(0.0), max_velocity_x(0.0),min_velocity_y(0.0), max_velocity_y(0.0),
    min_yawrate(0.0), max_yawrate(0.0)
{
}

Omnidirectional_DWAPlanner::Window::Window(const double min_v_x, const double max_v_x ,const double min_v_y ,const double max_v_y, const double min_y, const double max_y)
    : min_velocity_x(min_v_x), max_velocity_x(max_v_x), min_velocity_y(min_v_y) , max_velocity_y(max_v_y) , 
    min_yawrate(min_y), max_yawrate(max_y)
{
}

// 用于接收终点信息
void Omnidirectional_DWAPlanner::clickCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  // 获取全局终点信息
  global_goal = *msg;

  // 获取规划点
  lastendPoint[0] = msg->pose.position.x;
  lastendPoint[1] = msg->pose.position.y;

  // 获取到终点之后记录当前的时间
  end_point_time = ros::Time::now();
  updateEndFlag = true;
  // 长度归0
  all_length = 0;
  // 
  run_numbers = 0;

  // 
  run_times = 0;
}

void Omnidirectional_DWAPlanner::local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  local_map = *msg;
  // 获取地图数据
  getMapdata();
  local_map_updated = true;
}

// void motionPlan::localizationCallback(const robot_communication::localizationInfoBroadcastConstPtr &msg)
// {
//   // 获取起点等规划消息
//   localData.xPosition = msg->xPosition;
//   localData.yPosition = msg->yPosition;
//   localData.xSpeed = msg->xSpeed;
//   localData.ySpeed = msg->ySpeed;
//   localData.xAccel = msg->xAccel;
//   localData.yAccel = msg->yAccel;
//   localData.chassisGyro = msg->chassisGyro;
//   localData.chassisAngle = msg->chassisAngle;

//   startPoint[0] = msg->xPosition;
//   startPoint[1] = msg->yPosition;

//   if(fabs((startPoint-endPoint).norm()) <= 0.1)
//   {
//     has_arrived_end = true;
//   }
//   else{
//     has_arrived_end = false;
//   }

//   getStartFlag = true; // 获取到起点标志位
// }

void Omnidirectional_DWAPlanner::odom_callback(const robot_communication::localizationInfoBroadcastConstPtr &msg)
{
  // 获取当前的速度反馈信息
  current_velocity.linear.x = msg->xSpeed;
  current_velocity.linear.y = msg->ySpeed;
  current_velocity.angular.z = msg->chassisGyro;

  // 获取位置信息 还有角度信息
  nowposition[0] = msg->xPosition;
  nowposition[1] = msg->yPosition;
  nowposition[2] = msg->chassisAngle;

  odom_updated = true;
}

// 路径回调函数
void Omnidirectional_DWAPlanner::pathCallback(const nav_msgs::PathConstPtr &path)
{
  nav_msgs::Path tempath = *path;

  // 获取当前路径点数目
  path_nodes_num = tempath.poses.size();
  // 判断路径是不是空的
  if(path_nodes_num <= 0)
  {
    return;
  }

  // 当两次路径点数目没有发生改变，认为当前的终点没有发生变化，还是原来的路径
  if(path_nodes_num == last_path_nodes_num)
  {
    // 路径没有更新
    update_path = false;
  }
  if(path_nodes_num != last_path_nodes_num)
  {
    // 路径更新了
    update_path = true;
  }

  // 路径更新了就把新的路径放进来
  if(update_path == true)
  {
    // 跟随路径清空
    trajpath.clear();
    // 更新
    visitPath.clear();
    // 对跟随路径进行赋值
    for (int i = 0; i < path_nodes_num; i++)
    {
      // 临时路径点用于数据类型转换
      // 设定为0代表新的路径所有点没有被访问过
      Vector2d tempoint =Vector2d (tempath.poses[i].pose.position.x,
                                   tempath.poses[i].pose.position.y);
      // 添加到跟随路径中
      trajpath.push_back(tempoint);
      visitPath.push_back(make_pair(0,tempoint));
    }

    firststartPoint[0] = tempath.poses[0].pose.position.x;
    firststartPoint[1] = tempath.poses[1].pose.position.x;
  }

  // 当没更新的时候
  if(update_path == false)
  {
    // 定义临时终点
    Vector2d tempgoal;
    // 得出位置角度
    Vector2d tempnow = Vector2d(nowposition[0],nowposition[1]);
    tempgoal = caLocalGoalPosition(trajpath,tempnow,TEMP_GOAL_RADIUS);
    // 设定水平向量
    Vector2d horizontal = Vector2d(1.0,0);
    double angle =  calVectorAngle(horizontal,tempgoal-tempnow);
    // 使用templocal中最后一个担任临时终点
    localposition[0] = tempgoal[0];
    localposition[1] = tempgoal[1];
    // 获取角度
    localposition[2] = angle;
    local_goal_subscribed = true;
  }
  vector<Vector2d> tempvec;
  tempvec.push_back(Vector2d(localposition[0],localposition[1]));
  visual_VisitedNode(localgoalPub,tempvec,1,0,1,0.5,5);

  // 获取机器人的速度，用作目标速度
  GetRobotVelocity(nowposition,localposition,TARGET_VELOCITY);

  // 获取当前路径点的数目用作比较
  last_path_nodes_num = path_nodes_num;
}

// 获取机器人的速度
// nowpoint  当前位置
// endpoint  终点位置
void Omnidirectional_DWAPlanner::GetRobotVelocity(Vector3d nowpoint,Vector3d endpoint,Vector3d &localvelocity)
{
  Vector2d tempvelocity;

  // 根据临时终点计算目标速度 先计算线速度 再计算角度
  tempvelocity[0] = endpoint[0] - nowpoint[0];
  tempvelocity[1] = endpoint[1] - nowpoint[1];

  // 获取世界坐标系下的速度
  worldVel.Vx = tempvelocity[0];
  worldVel.Vy = tempvelocity[1];
  // 获取当前角度
  carVel.yaw = nowpoint[2];
  // 全局速度转换成局部速度
  vel_transform::GlobalVelocityToLocalVector(&carVel,&worldVel);

  // 限制速度矢量
  LIMIT_VECTOR_Velocity(carVel.Vx,carVel.Vy,MAX_VELOCITY_X);
  localvelocity[0] = carVel.Vx;
  localvelocity[1] = carVel.Vy;

  // 定义临时终点和临时起点
  Vector2d tempgoal = Vector2d(endpoint[0],endpoint[1]);
  Vector2d tempnow = Vector2d(nowpoint[0],nowpoint[1]);
  // 求解角速度，这里按照路径长度/合成速度解算=时间 再用角度/时间得到角速度
  double distance = calPointLength(tempnow,tempgoal);
  // 求解时间
  double use_time = distance/sqrt(pow(carVel.Vx,2)+pow(carVel.Vy,2));

  Vector2d positive_vel = Vector2d(1.0,0.0);
  double setcarangle = calVectorAngle(positive_vel,tempvelocity);

  // 计算角度度
  pidFollow.gyro_pid.SetPoint = setcarangle;
  pidFollow.gyro_pid.feedPoint = nowpoint[2];
  Deal_Super_Circle(&pidFollow.gyro_pid.SetPoint,&pidFollow.gyro_pid.feedPoint);
  pidFollow.PIDFloatPositionCal(&pidFollow.gyro_pid);

  // 限制幅度
  pidFollow.gyro_pid.OutPoint = LIMIT(pidFollow.gyro_pid.OutPoint,-M_PI,M_PI);
  localvelocity[2] = pidFollow.gyro_pid.OutPoint;
  localvelocity[2] = 0;
}

void Omnidirectional_DWAPlanner::Deal_Super_Circle(double *setangle,double *feedangle)
{
  /* 
    因为当前的车的临界值是±PI，所以在达到这个角度的时候车只会按照
    正常的累积误差计算
  */
  float delta = *feedangle - *setangle;
  if(delta >= -M_PI && delta <= M_PI)
  {
    *feedangle = 0.0f;
    *setangle = delta;
  }
  else if(delta > M_PI)
  {
    *feedangle = 0.0f;
    *setangle = -M_PI*2 + delta;
  }
  else if(delta < -M_PI)
  {
    *feedangle = 0.0f;
    *setangle = M_PI*2+delta;
  }

  *setangle = -*setangle;
  *feedangle = 0.0f; 
}

// 计算向量的模长
double Omnidirectional_DWAPlanner::calVectorModLen(Vector2d vector1)
{
  return (sqrt(vector1[0]*vector1[0]+vector1[1]*vector1[1]));
}

// 用来求解当前位置点以一定半径相交的路径点
// path 输入路径
// radius 搜索半径
// updatepath 路径是否更新
// return 局部终点
Vector2d Omnidirectional_DWAPlanner::caLocalGoalPosition(vector<Vector2d> path,Vector2d nowpoint,double radius)
{
  Vector2d localgoal;
  static int last_point_num;
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

  // 跟随路径点
  int traj_num = 0;
  for (int i = now_num+1; i < path_length_num; i++)
  {
    // 计算距离
    double distance = calPointLength(visitPath[now_num].second,visitPath[i].second);
    // 距离约束并且一定不能访问过
    if(distance >= radius && distance < radius + 0.3 && visitPath[i].first == 0)
    {
      traj_num = i;
      break;
    }
  }

  // 路径更新
  if(update_path == true)
  {
    last_point_num = 0;
  }

  // 等于上一个
  if(traj_num == 0)
  {
    // 获取上一个点
    traj_num = last_point_num;
  }

  // 之前的点设定为1
  for (int i = 0; i < traj_num; i++)
  {
    visitPath[i].first = 1;
  }

  // 确定下一个点的采样位置
  if(traj_num < path_length_num-1)
  {
    localgoal = path[traj_num];
  }
  else
  {
    localgoal = path[path_length_num-1];
  }

  // 判断得出的局部终点到最后终点的位置
  double lastdis = calPointLength(localgoal,path[path_length_num-1]);
  if(lastdis <= radius+0.3)
  {
    localgoal = path[path_length_num-1]; 
  } 

  // 获取点
  last_point_num = traj_num;
  return localgoal;
}

// 用来对速度进行限制,因为机器人是全向移动的,如果直接通过数值对x y速度限制会产生畸变
// 所以需要先合成,再进行限制
// vel_x 输入的x方向上的速度
// vel_y 输入的y方向上的速度
// limit_velocity 限制的速度
void Omnidirectional_DWAPlanner::LIMIT_VECTOR_Velocity(double &vel_x,double &vel_y,double limit_velocity)
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

// 计算单位向量
Vector2d Omnidirectional_DWAPlanner::calUnitvector(Vector2d unitv)
{
  // 计算单位向量
  unitv = unitv * 1.0f/(sqrt(pow(abs(unitv[0]),2)+pow(abs(unitv[1]),2)));
  return unitv;
}

// 计算向量之间的夹角
double Omnidirectional_DWAPlanner::calVectorAngle(Vector2d vector1,Vector2d vector2)
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
double Omnidirectional_DWAPlanner::calPointLength(Vector2d vector1,Vector2d vector2)
{
  return (sqrt((vector1[0]-vector2[0])*(vector1[0]-vector2[0])+(vector1[1]-vector2[1])*(vector1[1]-vector2[1])));
}

vector<Omnidirectional_DWAPlanner::State> Omnidirectional_DWAPlanner::dwa_planning(
    Eigen::Vector3d goal,
    vector<vector<float>> obs_list)
{
  // 所有的跟随轨迹
  vector<vector<State>> trajectories;
  // 最好的跟随轨迹集合
  vector<pair<float,vector<State>>> trajectorpaths;
  // 最好的跟随轨迹
  vector<State> best_traj;
  // 目标速度
  Vector2d velocity_target = Vector2d(TARGET_VELOCITY[0],TARGET_VELOCITY[1]);

  // 障碍物代价容器
  vector<float> vec_obs_cost;
  // 到终点代价容器
  vector<float> vec_goal_cost;
  // 速度代价容器
  vector<float> vec_speed_cost;

  float min_cost = 1e6;
  float min_obs_cost = min_cost;
  float min_goal_cost = min_cost;
  float min_speed_cost = min_cost;

  geometry_msgs::Twist temp_velocity;
  temp_velocity.linear.x = TARGET_VELOCITY[0];
  temp_velocity.linear.y = TARGET_VELOCITY[1];
  temp_velocity.angular.z = 0;
  Window target_velocity_window = calc_dynamic_window(temp_velocity);
  // 进行原DWA算法的测试
  // Window target_velocity_window = calc_dynamic_window(current_velocity);

  ros::Time calobstacle_time_start = ros::Time::now();
  // 对速度状态空间采样
  for (float vx = target_velocity_window.min_velocity_x; vx <= target_velocity_window.max_velocity_x; vx += VELOCITY_RESOLUTION)
  {
    for (float vy = target_velocity_window.min_velocity_y; vy <= target_velocity_window.max_velocity_y; vy += VELOCITY_RESOLUTION)
    {
      // 定义采样速度范围
      Vector2d sample_Velocity = Vector2d(vx,vy);
      LIMIT_VECTOR_Velocity(sample_Velocity[0],sample_Velocity[1],MAX_VELOCITY_X);
      State state(0.0, 0.0, 0.0, 0,0,0);
      // State state(0.0, 0.0, 0.0, current_velocity.linear.x,current_velocity.linear.y,0);
      vector<State> traj;
      // 对速度和角速度积分预测机器人位置
      for (float t = 0; t <= PREDICT_TIME; t += DT)
      {
        motion(state, sample_Velocity[0], sample_Velocity[1], 0);
        traj.push_back(state);
      }

      float obstacle_cost;
      // 不使用ESDF
      if(!IS_USE_ESDF)
      {
        obstacle_cost = OBSTACLE_COST_GAIN * calc_obstacle_cost(traj,obs_list);
      }
      else  //使用ESDF
      {
        obstacle_cost = OBSTACLE_COST_GAIN * calc_obstacle_cost(traj)*resolution;
        // obstacle_cost = OBSTACLE_COST_GAIN * calc_obstacle_cost(traj);
      }

      if(obstacle_cost < 1e6)
      {
        // 计算到终点的代价
        float to_goal_cost = TO_GOAL_COST_GAIN * calc_to_goal_cost(traj, goal);
        // 计算速度代价
        float speed_cost = SPEED_COST_GAIN * calc_speed_cost(traj, TARGET_VELOCITY);
        // 计算最终总的代价，这里每*系数代表各项安全性指标  可以创新
        float final_cost = to_goal_cost + speed_cost + obstacle_cost;

        // 添加预测轨迹
        trajectories.push_back(traj);
        // 将代价和轨迹都添加进来
        trajectorpaths.push_back(make_pair(final_cost,traj));
        // 将代价值分别添加进来
        vec_obs_cost.push_back(obstacle_cost);
        vec_goal_cost.push_back(to_goal_cost);
        vec_speed_cost.push_back(speed_cost);
      }
    }
  }  
  obstacle_flag = true;
  ros::Time calobstacle_time_end = ros::Time::now();
  obstacle_time = (calobstacle_time_end-calobstacle_time_start).toSec();

  if(trajectories.size() < 1)
  {
    // 清空几个轨迹值
    trajectories.clear();
    trajectorpaths.clear();
    // 代价容器也清空
    vec_obs_cost.clear();
    vec_goal_cost.clear();
    vec_speed_cost.clear();

    // 重置代价最小值
    min_cost = 1e6;
    min_obs_cost = min_cost;
    min_goal_cost = min_cost;
    min_speed_cost = min_cost;
    // 定义速度窗口
    Window max_velocity_window(MIN_VELOCITY_X, MAX_VELOCITY_X,MIN_VELOCITY_Y,MAX_VELOCITY_Y,-MAX_YAWRATE, MAX_YAWRATE);
    // 对全部速度空间扫描 这是全部的动态窗口
    max_velocity_window.min_velocity_x = MIN_VELOCITY_X;
    max_velocity_window.max_velocity_x = MAX_VELOCITY_X;
    max_velocity_window.min_velocity_y = MIN_VELOCITY_Y;
    max_velocity_window.max_velocity_y = MAX_VELOCITY_Y;

    // 对速度状态空间采样
    for (float vx = max_velocity_window.min_velocity_x; vx <= max_velocity_window.max_velocity_x; vx += VELOCITY_RESOLUTION)
    {
      for (float vy = max_velocity_window.min_velocity_y; vy <= max_velocity_window.max_velocity_y; vy += VELOCITY_RESOLUTION)
      {
        // 定义采样速度范围
        Vector2d sample_Velocity = Vector2d(vx,vy);
        LIMIT_VECTOR_Velocity(sample_Velocity[0],sample_Velocity[1],MAX_VELOCITY_X);
        // 计算夹角
        double vel_angle = calVectorAngle(sample_Velocity,velocity_target);
      // 当角度太大的时候代价数值时不进行代价计算
        State state(0.0, 0.0, 0.0, 0,0,0);
        vector<State> traj;
        // 对速度和角速度积分预测机器人位置
        for (float t = 0; t <= PREDICT_TIME; t += DT)
        {
          motion(state, sample_Velocity[0], sample_Velocity[1], 0);
          traj.push_back(state);
        }

        float obstacle_cost;
        // 不使用ESDF
        if(!IS_USE_ESDF)
        {
          obstacle_cost = OBSTACLE_COST_GAIN * calc_obstacle_cost(traj,obs_list);
        }
        else  //使用ESDF
        {
          obstacle_cost = OBSTACLE_COST_GAIN * calc_obstacle_cost(traj)*resolution;
          // obstacle_cost = OBSTACLE_COST_GAIN * calc_obstacle_cost(traj);
        }

        Vector2i now_pos = Vector2i(0,0);
        int8_t now_pos_esdf = getESDFvalue(now_pos);
        
        if(vel_angle >= M_PI/3)
        {
          obstacle_cost = 1e6;
        }
        if(obstacle_cost < 1e6)
        {
          // 计算到终点的代价
          float to_goal_cost = TO_GOAL_COST_GAIN * calc_to_goal_cost(traj, goal)*(1-now_pos_esdf/100.0f);
          // 计算速度代价
          float speed_cost = SPEED_COST_GAIN * calc_speed_cost(traj, TARGET_VELOCITY)*(1-now_pos_esdf/100.0f);
          // 计算最终总的代价，这里每*系数代表各项安全性指标  可以创新
          float final_cost = to_goal_cost + speed_cost + obstacle_cost;

          // 添加预测轨迹
          trajectories.push_back(traj);
          // 将代价和轨迹都添加进来
          trajectorpaths.push_back(make_pair(final_cost,traj));
          // 将代价值分别添加进来
          vec_obs_cost.push_back(obstacle_cost);
          vec_goal_cost.push_back(to_goal_cost);
          vec_speed_cost.push_back(speed_cost);
        }
      }
    }
  }

  if(trajectories.size() < 1)
  {
    // 清空几个轨迹值
    trajectories.clear();
    trajectorpaths.clear();
    // 代价容器也清空
    vec_obs_cost.clear();
    vec_goal_cost.clear();
    vec_speed_cost.clear();

    // 重置代价最小值
    min_cost = 1e6;
    min_obs_cost = min_cost;
    min_goal_cost = min_cost;
    min_speed_cost = min_cost;
    // 定义速度窗口
    Window max_velocity_window(MIN_VELOCITY_X, MAX_VELOCITY_X,MIN_VELOCITY_Y,MAX_VELOCITY_Y,-MAX_YAWRATE, MAX_YAWRATE);
    // 对全部速度空间扫描 这是全部的动态窗口
    max_velocity_window.min_velocity_x = MIN_VELOCITY_X;
    max_velocity_window.max_velocity_x = MAX_VELOCITY_X;
    max_velocity_window.min_velocity_y = MIN_VELOCITY_Y;
    max_velocity_window.max_velocity_y = MAX_VELOCITY_Y;

    // 对速度状态空间采样
    for (float vx = max_velocity_window.min_velocity_x; vx <= max_velocity_window.max_velocity_x; vx += VELOCITY_RESOLUTION)
    {
      for (float vy = max_velocity_window.min_velocity_y; vy <= max_velocity_window.max_velocity_y; vy += VELOCITY_RESOLUTION)
      {
        // 定义采样速度范围
        Vector2d sample_Velocity = Vector2d(vx,vy);
        LIMIT_VECTOR_Velocity(sample_Velocity[0],sample_Velocity[1],MAX_VELOCITY_X);
        // 计算夹角
        double vel_angle = calVectorAngle(sample_Velocity,velocity_target);
        State state(0.0, 0.0, 0.0, 0,0,0);
        vector<State> traj;
        // 对速度和角速度积分预测机器人位置
        for (float t = 0; t <= PREDICT_TIME; t += DT)
        {
          motion(state, sample_Velocity[0], sample_Velocity[1], 0);
          traj.push_back(state);
        }

        float obstacle_cost;
        // 不使用ESDF
        if(!IS_USE_ESDF)
        {
          obstacle_cost = OBSTACLE_COST_GAIN * calc_obstacle_cost(traj,obs_list);
        }
        else  //使用ESDF
        {
          obstacle_cost = OBSTACLE_COST_GAIN * calc_obstacle_cost(traj,2)*resolution;
          // obstacle_cost = OBSTACLE_COST_GAIN * calc_obstacle_cost(traj);
        }

        Vector2i now_pos = Vector2i(0,0);
        int8_t now_pos_esdf = getESDFvalue(now_pos);

        if(vel_angle < M_PI/3 || vel_angle > 2*M_PI/3)
        {
          obstacle_cost = 1e6;
        }
        if(obstacle_cost < 1e6)
        {
          // 计算到终点的代价
          float to_goal_cost = TO_GOAL_COST_GAIN * calc_to_goal_cost(traj, goal)*(1-now_pos_esdf/100.0f);
          // 计算速度代价
          float speed_cost = SPEED_COST_GAIN * calc_speed_cost(traj, TARGET_VELOCITY)*(1-now_pos_esdf/100.0f);
          // 计算最终总的代价，这里每*系数代表各项安全性指标  可以创新
          float final_cost = to_goal_cost + speed_cost + obstacle_cost;
          // 添加预测轨迹
          trajectories.push_back(traj);
          // 将代价和轨迹都添加进来
          trajectorpaths.push_back(make_pair(final_cost,traj));
          // 将代价值分别添加进来
          vec_obs_cost.push_back(obstacle_cost);
          vec_goal_cost.push_back(to_goal_cost);
          vec_speed_cost.push_back(speed_cost);
        }
      }
    }
  }

  // 获取最小值
  // 获取跟随的路径数目
  int trajpath_num = trajectorpaths.size();
  
  int visual_num = 0;
  if(trajpath_num > 0)
  {
    // 在使用之前初始化一次
    min_cost = 1e6;
    min_obs_cost = min_cost;
    min_goal_cost = min_cost;
    min_speed_cost = min_cost;
    for (int i = 0; i < trajpath_num; i++)
    {
      // 筛选到终点的代价最小值
      // 筛选最小代价值
      if(min_cost > trajectorpaths[i].first)
      {
        // 传递最小值
        min_cost = trajectorpaths[i].first;
        // 获取最佳路径
        best_traj = trajectorpaths[i].second;
        // 获取各项最小值
        min_goal_cost = vec_goal_cost[i];
        min_obs_cost = vec_obs_cost[i];
        min_speed_cost = vec_speed_cost[i];
      }
    }
  }

  // 这是一个大的创新点，先不急确定如何修改
  // 因为车是长方形的，所以一旦有一部分进入障碍物，这样就无解，但是理论上是可以走出来的，所以加入判断算法，将车本身附近的障碍物设定为0

  if (min_cost == 1e6 || trajpath_num == 0)
  {

    cout<<"进入障碍物停车了!"<<endl;
    vector<State> traj;
    // 定义x方向速度 定义y方向速度
    double x_velocity = 0;
    double y_velocity = 0;

    // 状态传递
    State state(0.0, 0.0, 0.0, 
    x_velocity,y_velocity,0);
    // 状态跟随
    traj.push_back(state);
    best_traj = traj;
  }

  // ROS_INFO_STREAM("Cost: " << min_cost);
  // ROS_INFO_STREAM("- Goal cost: " << min_goal_cost);
  // ROS_INFO_STREAM("- Obs cost: " << min_obs_cost);
  // ROS_INFO_STREAM("- Speed cost: " << min_speed_cost);
  // ROS_INFO_STREAM("num of trajectories: " << trajectories.size());

  // ROS_INFO_STREAM("target vx: " << TARGET_VELOCITY[0]);
  // ROS_INFO_STREAM("target vy: " << TARGET_VELOCITY[1]);
  // ROS_INFO_STREAM("set vx: " << best_traj[0].velocity_x);
  // ROS_INFO_STREAM("set vy: " << best_traj[0].velocity_y);

  visualize_trajectories(trajectories, 0, 1, 0, 1, candidate_trajectories_pub);

  // 返回最佳状态
  return best_traj;
}

// 查找跟随轨迹上的点有没有障碍物
// trajs 轨迹
float Omnidirectional_DWAPlanner::Find_Traj_Obstacle(vector<State> trajs,Vector2d tempgoal)
{
  float cost;
  for (int j = 0; j < trajs.size(); j++)
  {
    /* code */
    double traj_x = trajs[j].x;
    double traj_y = trajs[j].y;

    Vector2d traj_position = Vector2d(traj_x,traj_y);
    // 计算到临时终点的距离
    double dis_trajpoint_goal = calPointLength(traj_position,tempgoal);
    // 当在范围内的时候
    if(dis_trajpoint_goal <= GOAL_THRESHOLD)
    {
      // 纪律当前位置
      int now_num = j;
      // 
      int has_obstacle = 0;
      // 遍历之前的点是否有障碍物，没有则将代价值设定为0
      for(int num = 0;num < now_num;num++)
      {
        // 获取当前位置
        Vector2d nowpos = Vector2d(trajs[num].x,trajs[num].y);
        // 障碍物判断，不是障碍物的时候
        if(isObstacle(nowpos))
        {
          has_obstacle ++;
        }
      }
      if(has_obstacle == 0)
      {
        cost = dis_trajpoint_goal;
      }
      if(has_obstacle > 0)
      {
        cost = 1e6;
      }
    }   
  }

  return cost;
}

void Omnidirectional_DWAPlanner::process(void)
{
  ros::Rate loop_rate(HZ);

  while (ros::ok())
  {
    if (local_map_updated && local_goal_subscribed && odom_updated)
    {
      geometry_msgs::Twist cmd_vel;
      // 实际机器人控制的速度指令
      robot_communication::chassisControl controlVel;      //全局运动数据
      Vector2d localpoint = Vector2d(nowposition[0],nowposition[1]);
      if (calPointLength(localpoint,lastendPoint) > GOAL_THRESHOLD)
      {
        local_map_updated = false;
        ros::Time start = ros::Time::now();
        vector<State> best_traj;
        // 不使用欧几里得距离场
        if(!IS_USE_ESDF)
        {
          vector<vector<float>> obs_list;
          obs_list = raycast();
          // // 获取最好的状态去执行
          best_traj = dwa_planning(localposition,obs_list);
        }
        else  //使用欧几里得距离场
        {
          best_traj = dwa_planning(localposition);
        }
        ros::Time end = ros::Time::now();
        once_time = (end-start).toSec()*1000;

        // 终点更新的情况下累加
        if(updateEndFlag == true)
        {
          all_length += calPointLength(localpoint,lastpoint);
          // if(obstacle_flag == true)
          // {
          //   obstacle_flag = false;
          //   run_numbers ++;
          //   run_times += (obstacle_time*1000);
          // }
          run_numbers ++;
          run_times += once_time;
        }
        // 速度输出
        cmd_vel.linear.x = best_traj[0].velocity_x;
        cmd_vel.linear.y = best_traj[0].velocity_y;

        cmd_vel.linear.x = LIMIT(cmd_vel.linear.x,-0.5,0.5);
        cmd_vel.linear.y = LIMIT(cmd_vel.linear.y,-0.5,0.5);

        cmd_vel.angular.z = TARGET_VELOCITY[2];

        Vector2d velocity_acl = Vector2d(best_traj[0].velocity_y,best_traj[0].velocity_x);
        // LIMIT_VECTOR_Velocity(velocity_acl[0],velocity_acl[1],1.0);

        controlVel.xSpeed = velocity_acl[0];
        controlVel.ySpeed = velocity_acl[1];
        controlVel.chassisGyro = TARGET_VELOCITY[2];

        // 将筛选出来的路径可视化
        visualize_trajectory(best_traj, 1, 0, 0, selected_trajectory_pub);

        lastpoint = localpoint; //值传递
      }
      else
      {
        if(updateEndFlag == true)
        {
          updateEndFlag = false;  //清0代表当前到达终点
          ROS_WARN("start point is: %f %f ",firststartPoint[0],firststartPoint[1]);
          ROS_WARN("end point is: %f %f ",lastendPoint[0],lastendPoint[1]);
          ROS_WARN("all run time is: %f ms",run_times);
          ROS_WARN("all run numbers is: %d ",run_numbers);
          ROS_WARN("all length is: %f m",all_length);
        }
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        controlVel.xSpeed = 0;
        controlVel.ySpeed = 0;
        controlVel.chassisGyro = 0;
      }
      // 发布仿真的速度
      if(IS_SIM)
      {
        chassCtlPub.publish(cmd_vel);
      }
      else
      {
        chassCtlPub.publish(controlVel);
      }
      // 速度清0
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;

      controlVel.xSpeed = 0;
      controlVel.ySpeed = 0;
      controlVel.chassisGyro = 0;

      odom_updated = false;
    }
    else
    {
      if (!local_goal_subscribed)
      {
        ROS_WARN_THROTTLE(1.0, "Local goal has not been updated");
      }
      if (!odom_updated)
      {
        ROS_WARN_THROTTLE(1.0, "Odom has not been updated");
      }
      if (!local_map_updated)
      {
        ROS_WARN_THROTTLE(1.0, "Local map has not been updated");
      }
    }

    // ROS_WARN("dwa run time is:%f us",(end-start).toSec() * 1000000.0);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

Omnidirectional_DWAPlanner::Window Omnidirectional_DWAPlanner::calc_dynamic_window(const geometry_msgs::Twist &cur_velocity)
{
  Window window(MIN_VELOCITY_X, MAX_VELOCITY_X,MIN_VELOCITY_Y,MAX_VELOCITY_Y,-MAX_YAWRATE, MAX_YAWRATE);

  // 当x 和 y的线速度都大于0的时候
  window.min_velocity_x = max((cur_velocity.linear.x - MAX_ACCELERATION * 5 * DT), MIN_VELOCITY_X);
  window.max_velocity_x = min((cur_velocity.linear.x + MAX_ACCELERATION * 5 * DT), MAX_VELOCITY_X);
  window.min_velocity_y = max((cur_velocity.linear.y - MAX_ACCELERATION * 5 * DT), MIN_VELOCITY_Y);
  window.max_velocity_y = min((cur_velocity.linear.y + MAX_ACCELERATION * 5 * DT), MAX_VELOCITY_Y);
  return window;
}

float Omnidirectional_DWAPlanner::calc_to_goal_cost(const vector<State> &traj, const Eigen::Vector3d &goal)
{
  float last_cost = 0.0;
  for (int i = 0; i < traj.size(); i++)
  {
    /* code */
    Eigen::Vector3d position(traj[i].x, traj[i].y,traj[i].yaw);
    float cost = (position.segment(0, 2) - goal.segment(0, 2)).norm();
    last_cost += cost;
  }
  // 计算轨迹上最后一个点到临时终点的代价
  // Eigen::Vector3d position(traj[traj.size()-1].x, traj[traj.size()-1].y,traj[traj.size()-1].yaw);
  // last_cost = (position.segment(0, 2) - goal.segment(0, 2)).norm();  
  return last_cost;
}

// 计算速度的夹角作为代价值
float Omnidirectional_DWAPlanner::calc_speed_cost(const vector<State> &traj, const Vector3d target_velocity)
{
  // 跟随速度向量
  Vector2d trajVelocity = Vector2d(traj[0].velocity_x,traj[0].velocity_y); 
  // 目标速度向量
  Vector2d targVelocity = Vector2d(target_velocity[0],target_velocity[1]);

  // 计算夹角
  // float angle = float(abs(calVectorAngle(trajVelocity,targVelocity)));
  // 计算代价值
  float cost = float(calPointLength(trajVelocity,targVelocity));
  return cost;
}

// 判断当前点位置是否有障碍物
bool Omnidirectional_DWAPlanner::isObstacle(Vector2d position,const vector<vector<float>> &obs_list)
{
  bool is_obstacle = false;
  for(const auto &obs : obs_list)
  {
    float dist = sqrt((position[0] - obs[0]) * (position[0] - obs[0]) + (position[1] - obs[1]) * (position[1] - obs[1]));
    if (dist <= local_map.info.resolution)
    {
      is_obstacle = true;
      break;
    }
  }
  return is_obstacle;
}

// 判断当前点位置是否有障碍物
bool Omnidirectional_DWAPlanner::isObstacle(Vector2d position)
{
  bool is_obstacle = false;
  // 转换为栅格坐标
  Vector2i gridmap = worldToMap(position);
  // 判断是否被占据 或者说是不是障碍物
  int idx = gridmap[0];
  int idy = gridmap[1];

  return (idx < grid_map_x && idy < grid_map_y && (mapdata[idx + idy * grid_map_x] == 100));
}

// 计算到障碍物的距离数值
float Omnidirectional_DWAPlanner::calc_obstacle_cost(vector<State> &traj,const vector<vector<float>> &obs_list)
{
  float cost = 0.0;
  float min_dist = 1e3;
  for (const auto &state : traj)
  {
    for (const auto &obs : obs_list)
    {
      float dist = sqrt((state.x - obs[0]) * (state.x - obs[0]) + (state.y - obs[1]) * (state.y - obs[1]));
      if (dist <= local_map.info.resolution)
      {
        cost = 1e6;
        return cost;
      }
      min_dist = min(min_dist, dist);
    }
  }
  cost = 1.0 / min_dist;
  return cost;
}

// 获取当前点的势场数值
int8_t Omnidirectional_DWAPlanner::getESDFvalue(Vector2i gridpt)
{
  // 判断是否被占据 或者说是不是障碍物
  int idx = gridpt[0];
  int idy = gridpt[1];

  return (mapdata[idx + idy * grid_map_x]);
}

// 获取当前点的势场数值
int8_t Omnidirectional_DWAPlanner::getESDFvalue(Vector2d worldpt)
{
  Vector2i gridpt = worldToMap(worldpt);
  // 判断是否被占据 或者说是不是障碍物
  int idx = gridpt[0];
  int idy = gridpt[1];

  return (mapdata[idx + idy * grid_map_x]);
}

// 计算轨迹上的欧几里德距离场数值
float Omnidirectional_DWAPlanner::calc_obstacle_cost(vector<State> &traj,int model)
{
  float cost = 0.0;
  // Vector2d position = Vector2d(traj[traj.size()-1].x,traj[traj.size()-1].y);
  // cost = getESDFvalue(position);
  int traj_num = 0;
  for (const auto &state : traj)
  {
    traj_num ++;
    // 转换为栅格坐标系
    Vector2i grid_position = worldToMap(Vector2d(state.x,state.y));
    int8_t esdfvalue = getESDFvalue(grid_position);
    if(esdfvalue < 94)
    {
      esdfvalue = 0;
    }
    cost += esdfvalue;
    // 有障碍物的时候
    if (esdfvalue == 100)
    {
      cost = 1e8;
      return cost;
    }
  }

  if(model == 1)
  {
    return cost/traj_num;  
  }
  if(model == 2)
  {
    return cost;  
  }
}

// 机器人的运动模型
void Omnidirectional_DWAPlanner::motion(State &state, const double vx,const double vy, const double yawrate)
{
  state.yaw += yawrate * DT;
  state.x += vx * cos(state.yaw) * DT - vy*sin(state.yaw)*DT;
  state.y += vx * sin(state.yaw) * DT + vy*cos(state.yaw)*DT;
  state.velocity_x = vx;
  state.velocity_y = vy;
  state.yawrate = yawrate;
}

// 获取地图
void Omnidirectional_DWAPlanner::getMapdata(void)
{
  // 获取栅格地图的尺寸
  grid_map_x = local_map.info.width;
  grid_map_y = local_map.info.height;

  // 获取地图分辨率
  resolution = local_map.info.resolution;
  resolution_inv = 1.0/local_map.info.resolution;

  origin_x = grid_map_x/2.0;
  origin_y = grid_map_y/2.0;

	// 初始化一个数组,按照XYZ的大小去初始化数组
	mapdata = new int8_t[grid_map_x * grid_map_y]; // 为将地图转化为8进制栅格地图作准备
	// 内存处理,清空数组
	memset(mapdata, 0, grid_map_x * grid_map_y * sizeof(int8_t));

  // 获取地图
  for(int x=0;x<grid_map_x;x++){
    for(int y=0;y<grid_map_y;y++)
    {
      // 设定地图参数
      mapdata[x+y*grid_map_x] = local_map.data[x+y*grid_map_x];
    }
  }
}

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
Vector2d Omnidirectional_DWAPlanner::mapToWorld(Vector2i mapt) const
{
  double wx,wy;
  wx = (mapt[0] + 0.5) * resolution;
  wy = (mapt[1] + 0.5) * resolution;

  return Vector2d(wx,wy);
}

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
Vector2i Omnidirectional_DWAPlanner::worldToMap(Vector2d worldpt) const
{
  int mx,my;

  mx = (int)(1.0 * worldpt[0] * resolution_inv + origin_x);
  my = (int)(1.0 * worldpt[1] * resolution_inv + origin_y);

  return Vector2i(mx,my);
}

// 获取障碍物的坐标信息
// 按照圆形范围获取障碍物坐标数据
vector<vector<float>> Omnidirectional_DWAPlanner::raycast()
{
  vector<vector<float>> obs_list;
  // 以圆的形式进行周围的障碍物搜索判断
  // for (float angle = -M_PI; angle <= M_PI; angle += ANGLE_RESOLUTION)
  // {
  //   for (float dist = 0.0; dist <= MAX_DIST; dist += local_map.info.resolution)
  //   {
  //     float x = dist * cos(angle);
  //     float y = dist * sin(angle);
  //     int i = floor(x / local_map.info.resolution + 0.5) + local_map.info.width * 0.5;
  //     int j = floor(y / local_map.info.resolution + 0.5) + local_map.info.height * 0.5;
  //     if ((i < 0 || i >= local_map.info.width) || (j < 0 || j >= local_map.info.height))
  //     {
  //       break;
  //     }
  //     if (local_map.data[j * local_map.info.width + i] == 100)
  //     {
  //       vector<float> obs_state = {x, y};
  //       obs_list.push_back(obs_state);
  //       break;
  //     }
  //   }
  // }

  for (float x_axis = -MAX_DIST; x_axis <= MAX_DIST; x_axis += local_map.info.resolution)
  {
    for (float y_axis = -MAX_DIST; y_axis <= MAX_DIST; y_axis += local_map.info.resolution)
    {
      int i = floor(x_axis / local_map.info.resolution + 0.5) + local_map.info.width * 0.5;
      int j = floor(y_axis / local_map.info.resolution + 0.5) + local_map.info.height * 0.5;
      if ((i < 0 || i >= local_map.info.width) || (j < 0 || j >= local_map.info.height))
      {
        break;
      }
      if (local_map.data[j * local_map.info.width + i] == 100)
      {
        vector<float> obs_state = {x_axis, y_axis};
        obs_list.push_back(obs_state);
        break;
      }
    }
  }
  
  return obs_list;
}

void Omnidirectional_DWAPlanner::visualize_trajectories(const vector<vector<State>> &trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher &pub)
{
  visualization_msgs::MarkerArray v_trajectories;
  int count = 0;
  const int size = trajectories.size();
  for (; count < size; count++)
  {
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.color.r = r;
    v_trajectory.color.g = g;
    v_trajectory.color.b = b;
    v_trajectory.color.a = 0.8;
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.id = count;
    v_trajectory.scale.x = 0.02;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    v_trajectory.pose = pose;
    geometry_msgs::Point p;
    for (const auto &pose : trajectories[count])
    {
      p.x = pose.x;
      p.y = pose.y;
      v_trajectory.points.push_back(p);
    }
    v_trajectories.markers.push_back(v_trajectory);
  }
  for (; count < trajectories_size;)
  {
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::DELETE;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.id = count;
    v_trajectories.markers.push_back(v_trajectory);
    count++;
  }
  pub.publish(v_trajectories);
}

void Omnidirectional_DWAPlanner::visualize_trajectory(const vector<State> &trajectory, const double r, const double g, const double b, const ros::Publisher &pub)
{
  visualization_msgs::Marker v_trajectory;
  v_trajectory.header.frame_id = ROBOT_FRAME;
  v_trajectory.header.stamp = ros::Time::now();
  v_trajectory.color.r = r;
  v_trajectory.color.g = g;
  v_trajectory.color.b = b;
  v_trajectory.color.a = 0.8;
  v_trajectory.ns = pub.getTopic();
  v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
  v_trajectory.action = visualization_msgs::Marker::ADD;
  v_trajectory.lifetime = ros::Duration();
  v_trajectory.scale.x = 0.05;
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  v_trajectory.pose = pose;
  geometry_msgs::Point p;
  geometry_msgs::PoseStamped point;
  // 发布局部规划路径
  nav_msgs::Path local_path;
  // 发布局部规划的路径
  for (const auto &pose : trajectory)
  {
    p.x = pose.x;
    p.y = pose.y;
    v_trajectory.points.push_back(p);

    point.pose.position.x = pose.x;
    point.pose.position.y = pose.y;
    local_path.poses.push_back(point);
  }

  // 发布跟随路径
  pub.publish(v_trajectory);

  // 汽车基坐标系
  local_path.header.frame_id = ROBOT_FRAME;
  // 发布时间戳
  local_path.header.stamp = ros::Time::now();
  // 发布跟随以nav_msgs::Path的消息类型发布路径
  localPathPub.publish(local_path);
}

void Omnidirectional_DWAPlanner::visual_VisitedNode(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> visitnodes,
float a_set,float r_set,float g_set,float b_set,float length)
{
  visualization_msgs::Marker node_vis;
  node_vis.header.frame_id = "map";
  node_vis.header.stamp = ros::Time::now();

  node_vis.color.a = a_set;
  node_vis.color.r = r_set;
  node_vis.color.g = g_set;
  node_vis.color.b = b_set;
  node_vis.ns = "omnidirectional_dwa";

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

