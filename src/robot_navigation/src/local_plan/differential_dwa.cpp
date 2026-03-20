#include "local_plan/differential_dwa.h"

Differential_DWAPlanner::Differential_DWAPlanner(void)
    : local_nh("~"), local_goal_subscribed(false), scan_updated(false), local_map_updated(false), odom_updated(false)
{
  local_nh.param("HZ", HZ, {20});
  local_nh.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
  local_nh.param("TARGET_VELOCITY", TARGET_VELOCITY, {0.8});
  local_nh.param("MAX_VELOCITY", MAX_VELOCITY, {1.0});
  local_nh.param("MIN_VELOCITY", MIN_VELOCITY, {0.0});
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

  local_nh.param("TEMP_GOAL_RADIUS", TEMP_GOAL_RADIUS, {4});
  DT = 1.0 / HZ;
  
  // 跟随路径集合的发布
  candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
  // 选定的跟随路径
  selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
  // 发布局部规划路径
  localPathPub = local_nh.advertise<nav_msgs::Path>("/local_path",1);
  // 发布局部规划出来的速度
  chassCtlPub = local_nh.advertise<geometry_msgs::Twist>("/cmd_vel_auto", 1, true);

  // 查看局部终点
  localgoalPub = nh.advertise<visualization_msgs::Marker>("/local_goal",10);

  local_goal_sub = nh.subscribe("/local_goal", 1, &Differential_DWAPlanner::local_goal_callback, this);
  local_map_sub = nh.subscribe("/local_map_inflate", 1, &Differential_DWAPlanner::local_map_callback, this);
  odom_sub = nh.subscribe("/carto_odom", 1, &Differential_DWAPlanner::odom_callback, this);
  target_velocity_sub = nh.subscribe("/velocity_control", 1, &Differential_DWAPlanner::target_velocity_callback, this);
    // 订阅路径
  pathSub = nh.subscribe("/opt_path",10,&Differential_DWAPlanner::pathCallback,this);
  // pid参数初始化
  pidFollow.Init();
  // 角度跟随PID参数初始化
  pidFollow.SetPIDParameter(&pidFollow.gyro_pid,anglePID);
}

Differential_DWAPlanner::State::State(double _x, double _y, double _yaw, double _velocity, double _yawrate)
    : x(_x), y(_y), yaw(_yaw), velocity(_velocity), yawrate(_yawrate)
{
}

Differential_DWAPlanner::Window::Window(void)
    : min_velocity(0.0), max_velocity(0.0), min_yawrate(0.0), max_yawrate(0.0)
{
}

Differential_DWAPlanner::Window::Window(const double min_v, const double max_v, const double min_y, const double max_y)
    : min_velocity(min_v), max_velocity(max_v), min_yawrate(min_y), max_yawrate(max_y)
{
}

void Differential_DWAPlanner::local_goal_callback(const robot_communication::goalConstPtr &msg)
{
  local_goal = *msg;
  local_goal_subscribed = true;
}

void Differential_DWAPlanner::local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  local_map = *msg;
  local_map_updated = true;
}

void Differential_DWAPlanner::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
  current_velocity = msg->twist.twist;

  // 当前速度过小的时候，提供一个激励
  // 因为是全向移动底盘，计算一个合速度
  double velocity = sqrt(pow(current_velocity.linear.x,2)+pow(current_velocity.linear.y,2));
  if(abs(velocity) <= 0.2)
  {
    velocity = 0.2;
  }
  current_velocity.linear.x = velocity;

  odom_updated = true;
}

// 路径回调函数
void Differential_DWAPlanner::pathCallback(const nav_msgs::PathConstPtr &path)
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

  // // 获取机器人的速度，用作目标速度
  // GetRobotVelocity(nowposition,localposition,TARGET_VELOCITY);

  // 获取当前路径点的数目用作比较
  last_path_nodes_num = path_nodes_num;
}

// 计算向量之间的夹角
double Differential_DWAPlanner::calVectorAngle(Vector2d vector1,Vector2d vector2)
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
double Differential_DWAPlanner::calPointLength(Vector2d vector1,Vector2d vector2)
{
  return (sqrt((vector1[0]-vector2[0])*(vector1[0]-vector2[0])+(vector1[1]-vector2[1])*(vector1[1]-vector2[1])));
}

// 用来求解当前位置点以一定半径相交的路径点
// path 输入路径
// radius 搜索半径
// updatepath 路径是否更新
// return 局部终点
Vector2d Differential_DWAPlanner::caLocalGoalPosition(vector<Vector2d> path,Vector2d nowpoint,double radius)
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

// 计算单位向量
Vector2d Differential_DWAPlanner::calUnitvector(Vector2d unitv)
{
  // 计算单位向量
  unitv = unitv * 1.0f/(sqrt(pow(abs(unitv[0]),2)+pow(abs(unitv[1]),2)));
  return unitv;
}

// 用来对速度进行限制,因为机器人是全向移动的,如果直接通过数值对x y速度限制会产生畸变
// 所以需要先合成,再进行限制
// vel_x 输入的x方向上的速度
// vel_y 输入的y方向上的速度
// limit_velocity 限制的速度
void Differential_DWAPlanner::LIMIT_VECTOR_Velocity(double &vel_x,double &vel_y,double limit_velocity)
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

// 获取机器人的速度
// nowpoint  当前位置
// endpoint  终点位置
void Differential_DWAPlanner::GetRobotVelocity(Vector3d nowpoint,Vector3d endpoint,Vector3d &localvelocity)
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
  // LIMIT_VECTOR_Velocity(carVel.Vx,carVel.Vy,MAX_VELOCITY_X);
  LIMIT_VECTOR_Velocity(carVel.Vx,carVel.Vy,MAX_VELOCITY);
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

void Differential_DWAPlanner::Deal_Super_Circle(double *setangle,double *feedangle)
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

void Differential_DWAPlanner::target_velocity_callback(const geometry_msgs::TwistConstPtr &msg)
{
  // 目标速度也需要合成
  TARGET_VELOCITY = sqrt(pow(msg->linear.x,2)+pow(msg->linear.y,2));
  ROS_INFO_STREAM("target velocity was updated to " << TARGET_VELOCITY << "[m/s]");
}

std::vector<Differential_DWAPlanner::State> Differential_DWAPlanner::dwa_planning(
    Window dynamic_window,
    Eigen::Vector3d goal,
    std::vector<std::vector<float>> obs_list)
{
  float min_cost = 1e6;
  float min_obs_cost = min_cost;
  float min_goal_cost = min_cost;
  float min_speed_cost = min_cost;

  std::vector<std::vector<State>> trajectories;
  std::vector<State> best_traj;

  // 对速度上进行采样
  for (float v = dynamic_window.min_velocity; v <= dynamic_window.max_velocity; v += VELOCITY_RESOLUTION)
  {
    // 对角速度进行采样
    for (float y = dynamic_window.min_yawrate; y <= dynamic_window.max_yawrate; y += YAWRATE_RESOLUTION)
    {
      State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z);
      std::vector<State> traj;
      // 对速度和角速度积分预测机器人位置
      for (float t = 0; t <= PREDICT_TIME; t += DT)
      {
        motion(state, v, y);
        // 将预测状态添加进来
        traj.push_back(state);
      }
      trajectories.push_back(traj);
      
      // 计算到终点的代价
      float to_goal_cost = calc_to_goal_cost(traj, goal);
      // 计算速度代价
      float speed_cost = calc_speed_cost(traj, TARGET_VELOCITY);
      // 计算障碍物代价
      float obstacle_cost = calc_obstacle_cost(traj, obs_list);
      // 计算最终总的代价，这里每*系数代表各项安全性指标  可以创新
      float final_cost = TO_GOAL_COST_GAIN * to_goal_cost + SPEED_COST_GAIN * speed_cost + OBSTACLE_COST_GAIN * obstacle_cost;

      // 筛选出来最小的放在这
      if (min_cost >= final_cost)
      {
        min_goal_cost = TO_GOAL_COST_GAIN * to_goal_cost;
        min_obs_cost = OBSTACLE_COST_GAIN * obstacle_cost;
        min_speed_cost = SPEED_COST_GAIN * speed_cost;
        min_cost = final_cost;
        best_traj = traj;
      }
    }
  }
  ROS_INFO_STREAM("Cost: " << min_cost);
  ROS_INFO_STREAM("- Goal cost: " << min_goal_cost);
  ROS_INFO_STREAM("- Obs cost: " << min_obs_cost);
  ROS_INFO_STREAM("- Speed cost: " << min_speed_cost);
  ROS_INFO_STREAM("num of trajectories: " << trajectories.size());
  visualize_trajectories(trajectories, 0, 1, 0, 100, candidate_trajectories_pub);
  if (min_cost == 1e6)
  {
    std::vector<State> traj;
    State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z);
    traj.push_back(state);
    best_traj = traj;
  }
  return best_traj;
}

void Differential_DWAPlanner::process(void)
{
  ros::Rate loop_rate(HZ);

  while (ros::ok())
  {
    ROS_INFO("==========================================");
    double start = ros::Time::now().toSec();


    if (local_map_updated && local_goal_subscribed && odom_updated)
    {
      Window dynamic_window = calc_dynamic_window(current_velocity);

      // Eigen::Vector3d goal(local_goal.Position_x, local_goal.Position_y,local_goal.Position_yaw/180*M_PI);
      Eigen::Vector3d goal(localposition[0], localposition[1],localposition[2]);

      ROS_INFO_STREAM("local goal: (" << goal[0] << "," << goal[1] << "," << goal[2] / M_PI * 180 << ")");
      geometry_msgs::Twist cmd_vel;
      if (goal.segment(0, 2).norm() > GOAL_THRESHOLD)
      {
        cout<<"1 goal.segment(0, 2).norm():"<<goal.segment(0, 2).norm()<<endl;
        std::vector<std::vector<float>> obs_list;
        obs_list = raycast();
        local_map_updated = false;

        std::vector<State> best_traj = dwa_planning(dynamic_window, goal, obs_list);

        cmd_vel.linear.x = best_traj[0].velocity;
        cmd_vel.angular.z = best_traj[0].yawrate;
        visualize_trajectory(best_traj, 1, 0, 0, selected_trajectory_pub);
      }
      else
      {
        cout<<"2 goal.segment(0, 2).norm():"<<goal.segment(0, 2).norm()<<endl;
        cmd_vel.linear.x = 0.0;
        if (fabs(goal[2]) > TURN_DIRECTION_THRESHOLD)
        {
          cmd_vel.angular.z = std::min(std::max(goal(2), -MAX_YAWRATE), MAX_YAWRATE);
        }
        else
        {
          cmd_vel.angular.z = 0.0;
        }
      }
      ROS_INFO_STREAM("cmd_vel: (" << cmd_vel.linear.x << "[m/s], " << cmd_vel.angular.z << "[rad/s])");
      chassCtlPub.publish(cmd_vel);

      odom_updated = false;
    }
    else
    {
      std::cout<<"local_map_updated:"<<local_map_updated<<std::endl;
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
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO_STREAM("loop time: " << ros::Time::now().toSec() - start << "[s]");
  }
}

Differential_DWAPlanner::Window Differential_DWAPlanner::calc_dynamic_window(const geometry_msgs::Twist &cur_velocity)
{
  Window window(MIN_VELOCITY, MAX_VELOCITY, -MAX_YAWRATE, MAX_YAWRATE);
  window.min_velocity = std::max((current_velocity.linear.x - MAX_ACCELERATION * DT), MIN_VELOCITY);
  window.max_velocity = std::min((current_velocity.linear.x + MAX_ACCELERATION * DT), MAX_VELOCITY);
  window.min_yawrate = std::max((current_velocity.angular.z - MAX_D_YAWRATE * DT), -MAX_YAWRATE);
  window.max_yawrate = std::min((current_velocity.angular.z + MAX_D_YAWRATE * DT), MAX_YAWRATE);
  return window;
}

float Differential_DWAPlanner::calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal)
{
  Eigen::Vector3d last_position(traj.back().x, traj.back().y, traj.back().yaw);
  return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

// 计算速度代价数值
float Differential_DWAPlanner::calc_speed_cost(const std::vector<State> &traj, const float target_velocity)
{
  float cost = fabs(target_velocity - fabs(traj[traj.size() - 1].velocity));
  return cost;
}

// 计算到障碍物的距离数值 --wrc 2026-1-7
float Differential_DWAPlanner::calc_obstacle_cost(std::vector<State> &traj,const std::vector<std::vector<float>> &obs_list)
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
      min_dist = std::min(min_dist, dist);
    }
  }
  cost = 1.0 / min_dist;
  return cost;
}

// 机器人的运动模型
void Differential_DWAPlanner::motion(State &state, const double velocity, const double yawrate)
{
  state.yaw += yawrate * DT;
  state.x += velocity * std::cos(state.yaw) * DT;
  state.y += velocity * std::sin(state.yaw) * DT;
  state.velocity = velocity;
  state.yawrate = yawrate;
}

// 通过激光雷达获取障碍物数据
std::vector<std::vector<float>> Differential_DWAPlanner::scan_to_obs()
{
  std::vector<std::vector<float>> obs_list;
  // 获取雷达扫描范围的最小角度
  float angle = scan.angle_min;
  // 进行最小范围的扫描
  for (auto r : scan.ranges)
  {
    // 获取障碍物的点集合
    float x = r * cos(angle);
    float y = r * sin(angle);
    std::vector<float> obs_state = {x, y};
    obs_list.push_back(obs_state);
    angle += scan.angle_increment;
  }
  return obs_list;
}


std::vector<std::vector<float>> Differential_DWAPlanner::raycast()
{
  std::vector<std::vector<float>> obs_list;
  for (float angle = -M_PI; angle <= M_PI; angle += ANGLE_RESOLUTION)
  {
    for (float dist = 0.0; dist <= MAX_DIST; dist += local_map.info.resolution)
    {
      float x = dist * cos(angle);
      float y = dist * sin(angle);
      int i = floor(x / local_map.info.resolution + 0.5) + local_map.info.width * 0.5;
      int j = floor(y / local_map.info.resolution + 0.5) + local_map.info.height * 0.5;
      if ((i < 0 || i >= local_map.info.width) || (j < 0 || j >= local_map.info.height))
      {
        break;
      }
      if (local_map.data[j * local_map.info.width + i] == 100)
      {
        std::vector<float> obs_state = {x, y};
        obs_list.push_back(obs_state);
        break;
      }
    }
  }
  return obs_list;
}

void Differential_DWAPlanner::visualize_trajectories(const std::vector<std::vector<State>> &trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher &pub)
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

void Differential_DWAPlanner::visualize_trajectory(const std::vector<State> &trajectory, const double r, const double g, const double b, const ros::Publisher &pub)
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


void Differential_DWAPlanner::visual_VisitedNode(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> visitnodes,
float a_set,float r_set,float g_set,float b_set,float length)
{
  visualization_msgs::Marker node_vis;
  node_vis.header.frame_id = "map";
  node_vis.header.stamp = ros::Time::now();

  node_vis.color.a = a_set;
  node_vis.color.r = r_set;
  node_vis.color.g = g_set;
  node_vis.color.b = b_set;
  node_vis.ns = "differential_dwa";

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



