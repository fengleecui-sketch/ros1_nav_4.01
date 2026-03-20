# 该包主要是写的全局路径规划算法、路径优化算法、局部规划算法以及路径跟随算法
## launch文件介绍
1. acl_run_rviz.launch 用来启动真实状态下的rviz
2. astar.launch 用来启动astar算法的静态仿真
3. cost_map.launch 用来启动ros包自带的代价地图包
4. DWA.launch 局部规划(目前效果不太好)
5. Hybrid_astar.launch 混合astar静态仿真
6. motionPlan_acl.launch 运动规划实车运行
7. motionPlan_sim.launch 运动规划仿真运行
8. nav_node.launch astar、JPS、hybrid astar静态算法对比
9. pure_pursuit 五次多项式规划轨迹跟随
10. rrt.launch rrt算法静态仿真
11. simulation.launch fast_planner 仿真(目前还没成功)
## 代码框架介绍
nav_app.cpp nav_app.h nav_node.cpp 中主要用于导航算法的静态仿真对比
motionPlan.cpp motionPlan.h 用来进行运动规划，motionPlan是一个总的控制节点，按照顺序执行全局规划、路径优化、路径跟随三部分算法内容，相当于单线程执行三部分内容，这样代码耦合性太高，不利于进行修改，后续将会把这积分内容分开实现
## motionPlan 运动规划实现过程避坑介绍:
**注意：因为尝试过应用fast planner，虽然失败了，但是半成品的代码在该导航包里，这里面需要nlopt库，nlopt库的链接在CMakeLists.txt中，在ubuntu下打开文件-->其他位置-->计算机-->搜索libnlopt，搜索结果中会显示位置，将其修改添加即可**
```c++
add_executable(nav_node src/nodes/nav_node.cpp ${ALL_SRCS} )
target_link_libraries(nav_node 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libnlopt.so  // nlopt库链接，不同的电脑不同，注意改变
  ${THIRD_PART_LIBRARIES} 
  ${OpenCV_LIBS}
  )
```
1. 定位在全局规划中充当起点的作用，定位消息订阅地图消息订阅和终点消息订阅，一定在地图初始化完毕之后且能接收到终点和起点(定位)消息之后开始尽心全局规划，否则会报错
2. 如果采用该代码结构框架**重点：src文件中_node.cpp和具体实现.cpp分开的结构进行代码构造的时候，如果需要从launch文件中获取参数，如果这样，需要在构造函数中设置私有节点进行传参**，代码如下：
```c++
// 在头文件class构建中按照以下代码构建
private:
  /* data */
  ros::NodeHandle motPlan;       //运动规划节点
  // 如果没有私有节点，launch文件中的参数加载不进来，目前还不知道为什么，但是一定要像这样使用
  ros::NodeHandle private_node;  // ros中的私有句柄

// 在.cpp的构造函数中
// 如果没有私有节点，launch文件中的参数加载不进来，目前还不知道为什么，但是一定要像这样使用
motionPlan::motionPlan() : private_node("~")
{
  // 初始化 发表话题和订阅话题使用motPlan
	// 获取参数使用private_node
  motionInit();
}

void motionPlan::motionInit(void)
{
  // 参数初始化
  startPoint = Eigen::Vector2d(-1.00f,-1.00f);
  endPoint = Eigen::Vector2d(-1.00f,-1.00f);

  // 订阅静态地图
  staticMap = motPlan.subscribe("/map",10,&motionPlan::staticMapCallback,this);
  // 订阅鼠标点击目标点
  clickSub = motPlan.subscribe("/clicked_point",10,&motionPlan::clickCallback,this);
  // 订阅定位消息
  localizationSub = motPlan.subscribe("/truth_pose_odom",10,&motionPlan::localizationCallback,this);

  // 发布话题
  // 规划路径发布
  oriPathPub = motPlan.advertise<nav_msgs::Path>("/ori_path",10);
  // 优化路径发布
  optPathPub = motPlan.advertise<nav_msgs::Path>("/opt_path",10);
  // 发布速度消息
  chassControl = motPlan.advertise<robot_communication::chassisControl>("/acl_velocity",1,true);

  // 访问节点发布
  visitNodesPub = motPlan.advertise<visualization_msgs::Marker>("/visitNodes",10);

  // 加载参数
  // 路径规划参数加载
  double astar_weight_g,astar_weight_h;
  int astar_heuristic,astar_glength;
  
  private_node.param("astar_weight/g",astar_weight_g,1.0);  //权重a值
  private_node.param("astar_weight/h",astar_weight_h,1.0);  //权重b值
  private_node.param("astar_heuristic/distance",astar_heuristic,0);    //0 1 2 3方法 欧氏距离 曼哈顿 切比学夫 对角线
  private_node.param("astar_glength/distance",astar_glength,0);

  astar_path_finder->setParams(astar_weight_g,astar_weight_h,astar_glength,astar_heuristic);

  // 路径优化参数加载
  double max_vel,max_acce;
  int order;
  private_node.param("minimum_snap/max_vel",max_vel,1.0);
  private_node.param("minimum_snap/max_acce",max_acce,1.0);
  private_node.param("minimum_snap/order",order,3);
  MinimumSnapFlow.setParams(order,max_vel,max_acce);
   
  // 路径跟随参数加载
  private_node.param("pid_follow/kp",paramSolver.kp,1.0);
  private_node.param("pid_follow/ki",paramSolver.ki,1.0);
  private_node.param("pid_follow/forwardDistance",paramSolver.forwardDistance,1.0);
  private_node.param("pid_follow/speedLimit",paramSolver.speedLimit,1.0);
}
```
3. motionPlan代码中没有局部规划算法，局部规划算法不会在motionPlan中进行部署
4. motionPlan_acl.launch 用于真实场景中的路径规划
   motionPlan_sim.launch 用于仿真情景中的路径规划
   **注意:motionPlan_acl和motionPlan_sim两个launch文件对应的是同一个应用程序motionPlan.cpp，但是launch文件中的参数略有不同，motionPlan.cpp目前的作用是用来测试局部规划算法，当然其中也包括全局规划的部分，具体需要测试全局规划还是局部规划可以根据个人要求进行修改，但是主要是一个测试平台，不算是一个针对具体场景的应用程序，需要注意，如果说有实际项目需要进行运动规划，建议按照motionPlan.cpp的思路重新写一个程序架构**

## 目前已经完成：全局规划算法、路径优化算法 待完成：局部规划算法、路径跟随算法
## 全局规划算法主要有：
### Astar算法：
可以去bilibili上的深蓝学院根据视频教程开源代码进行学习
[深蓝学院](https://www.bilibili.com/video/BV1hA411q7Db/?spm_id_from=333.788&vd_source=45d17a7fb904c6c80f292c82cbb3107d)
这里视频中的源码是面对三维环境的，该功能包已经成功进行降维到二维
**重点：** 代价函数的距离计算:f=g+h，g代表已经走过的距离，h代表到达终点需要走过的距离，程序中使用的是8连通域的规划，即机器人可以进行对角线移动，输入的地图为ros中的地图即可，这里需要简单说一下在nav_app.cpp中的代码：该代码将ROS地图先列后行的扫描方式转换为先行后列的扫描方式，这个算法在JPS和RRT*进行全局规划同样有应用
A*算法起源论文：astar在该功能包下
```c++
/*** 
 * @description: 将ros解析的地图转变为正常的先行后列的顺序，非常非常非常重要
 * @param 1 读取出来的地图的参数  一维数组
 * @param 2 转换为一维算法能用的普通一维数组
 * @param 3 地图的x方向尺寸
 * @param 4 地图的y方向尺寸
 * @return {*}
 */
void navSolution::NavtfGrid(const nav_msgs::OccupancyGrid::ConstPtr &data,std::vector<int> &map,int x_size,int y_size)
{
    map.resize(x_size*y_size);
    for(int i=0;i<x_size;i++)
    {
        for(int j=0;j<y_size;j++)
        {   /* 这行代码非常非常非常重要，将ros解析的地图转变为正常的先行后列的顺序 */
            map[i*y_size+j] = int(data->data[j*x_size+i]);
        }
    }   
}
```
### JPS算法：
学习地址同上，已经从三维降低到二维环境
**重点：** JPS算法实现的父类是Astar的类，然后启发式算法和Astar相同，这里降维的难点在于：强制邻居节点的确定，三维和二维情况下是无法通用的，目前这里二维环境下强制邻居节点的确认选择是从github上找到的开源方案，需要三维的用深蓝学院源码就可以，目前JPS的缺点就是依靠障碍物确定跳点，规划出来的路径距离障碍物很接近，如果平台运动响应不够，就会容易产生碰撞，之前找到一篇论文针对该缺点进行优化，后续准备使用一下，论文名字：Jump Point Search Plus Algorithm Based on Radar Simulation Target Path Planning
JPS算法起源论文：jump point search 以上两篇文献均在该功能包下
### RRT*算法：
学习地址忘了，网上开源很多，但是不是都能使用，该功能包已经整理好
**重点：** 基于采样的路径规划算法，改进空间很大，但是规划路径方法是根据概率方法来的，实用性很差，容易创新发论文比较容易
### Hybrid A*:已经更改并且适用于ubuntu20.04版本的Fast Planner代码git[链接](https://gitee.com/RobotFormation_Building/robot-algorithm-tutorial.git)
Hybrid A*目前已经实现，参照的是fast planner的开源代码，Fast planner使用的是三维的环境的混合Astar算法，所以按照三维的重新写了一遍，把维度降低到二维情况下，说几个注意事项：
**重点：**
1. 不同于传统astar算法，Hybrid A*算法考虑了运动平台的运动学数学模型进行全局规划，因此其生成的路径会比普通Astar算法要更适合运动平台进行运动
2. Fast planner创新性的提出了采取设定全局搜索深度的方法降低算法的时间复杂度，通俗来说就是化整为零，因为Fast planner针对的环境是未知环境，在未知环境中地图也未知规划整张地图的全局路径并没有什么必要，会提高算法对计算平台算力的需求，降低系统的实时性和快速性。搜索范围的设定根据当前可见地图范围进行设定，比如可见范围是方圆7m的区域，我们设定的范围就可以是7m或者小一点但是没必要过大。最终实现的效果就是很快
3. Fast planner在Hybrid A*算法中加入了系统实时速度和加速度的考虑，所以可以使得规划出来的路径能更好的适应当前的运动状态，所以这也是Fast planner的具有独特意义的创新点之一。因为考虑进去了实时的速度和加速度，所以如果使用的时候需要考虑这两点因素需要很准的惯性传感器
4. 根据当前这段时间的学习，凡是基于图搜索的Astar、JPS、Hybrid Astar、Dijstra等全局规划算法，其全局搜索速度是由所在硬件平台的算力决定的，目前很难通过改进算法本身大幅度提高搜索速度(这里指的是JPS相对于Astar的搜索速度提升)，基本上已经到瓶颈了，所以当前的研究重点已经转移到后端上了，包括对于规划出来的路径进行优化、局部规划、轨迹跟随等内容
5. Hybrid Astar部分的代码中需要注意的参数设定部分在使用之前需要加强了解，可以看Hybrid_astar.launch中的内容：
```c++
<node name="Hybrid_node" pkg="nav_lei" type="Hybrid_node" output="screen">
  <!-- // 规划器允许的最大时间长度，即规划出的路径时间不能超过max_tau_ -->
  <param name="search/max_tau" value="0.6" type="double"/>
  <!-- 路径规划初始时的最大时间跨度，一般情况下会比 max_tau 大，用于加快搜索速度 -->
  <param name="search/init_max_tau" value="0.8" type="double"/>
  <!-- 运动平台最大速度 -->
  <param name="search/max_vel" value="3.0" type="double"/>
  <!-- 运动平台最大加速度 -->
  <param name="search/max_acc" value="3.0" type="double"/>
  <!--控制路径规划中时间成本的权重，用于平衡时间成本和距离成本。值越大，时间成本就越重要 -->
  <param name="search/w_time" value="10.0" type="double"/>
  <!-- 控制路径规划的搜索深度，即在未来多少秒内进行路径规划。值越大，搜索深度就越深 -->
  <param name="search/horizon" value="20.0" type="double"/>
  <!-- 控制启发式函数中时间成本的权重 -->
  <param name="search/lambda_heu" value="5.0" type="double"/>
  <!-- 控制 A* 算法的搜索分辨率，用于控制搜索空间的大小。值越小，搜索空间就越大，但搜索速度就会变慢 -->
  <!-- 一般是地图分辨率 -->
  <param name="search/resolution_astar" value="0.05" type="double"/>
  <!-- 时间分辨率 -->
  <param name="search/time_resolution" value="0.8" type="double"/>
  <!-- 边缘速度，或者说具体边缘什么需要后续对算法进一步理解才能知道 -->
  <param name="search/margin" value="0.2" type="double"/>
  <!-- 最长的路径 -->
  <param name="search/allocate_num" value="100000" type="int"/>
  <!-- 检查路径点的数目 -->
  <param name="search/check_num" value="5" type="int"/>
</node>
```
6. 如果Hybrid Astar使用过程中出现了bug可能是因为存在某些问题还没解决，因为5月12日代码刚开始部署并且初步测试和应用
### 总结：
以上算法的输入输出接口已经写好：输入：是栅格地图和起点终点，有两种仿真方法可调整起点终点仿真，以及固定起点进行仿真，目前使用的是固定起点进行仿真。输出：规划的路径一个二维数组，可以通过终端查看算法的运行时间。以上的参数通过查看代码nav_app.cpp和rrt_node.cpp以及Astar_node.cpp可以看到
在看代码的时候，建议先从算法的初始化部分开始看起，全局算法初始化中涉及到栅格地图的初始化，以及算法需要提前设定的参数比如说选择哪种代价函数计算方法。其中世界坐标系到栅格坐标系和栅格坐标系到世界坐标系的转换部分，这部分与之前提到的地图转换是相关的，一定要注意
### 补充：后续可能会部署Hybrid A*，部署的目的是Hybrid A*能够更好的发挥全向移动底盘的运动优势

## 路径优化算法：
### Minimum_Snap算法：
深蓝学院有开源代码，但是代码主要是针对三维情况下的，目前工程中的代码是根据另一篇教程开源的二维代码，在bilibili上搜索[IR艾若机器人](https://www.bilibili.com/video/BV1yT4y1T7Eb/?spm_id_from=333.999.0.0&vd_source=45d17a7fb904c6c80f292c82cbb3107d)这里不只讲了minimum snap的原理及应用也讲了贝塞尔曲线的原理和应用十分全面很透彻，提供的开源代码也是针对二维情况下的
**重点：**minimum_snap对路径平滑的效果很好，越高阶效果越好但是缺点就是运算时间长，目前解决方案有一种，对于未知环境下，全局规划算法只规划当前可见空间内的路径，并提取关键点采用minimum snap进行优化，随着机器人移动进行SLAM建图，规划，优点是可以适应未知环境的规划，缺点就是当前二维平台没有成熟的SLAM建图方案进行测试。当前该功能包中使用的是分段优化，按照固定数量取点优化，缺点就是分段点处不连续。
### 贝塞尔算法：
**重点：**学习链接同minimum snap相同，贝塞尔的特点就是高阶可推出低阶，反之同样可以。缺点就是理论上越高阶优化效果越好，但是目前执行效果来看，不是这样的，不知道问题出现在哪了，以后改进一下，目前可以稳定运行的最高阶的是11阶。目前2、3、4阶的代码也有，效果不如11阶的好，但是11阶出现的问题是，11阶的原理是对于规划的路径每次取11个点进行分段，然后优化，在分段点出会出现不连续。
### 总结：
以上算法的输入输出接口已经写好：输入：规划出来的路径的二维数组。输出：优化好后的路径二维数组。

## 局部规划算法：
### DWA算法:
**重点：**DWA算法主要思想在机器人的运动空间中定义一个窗口，该窗口代表了机器人在下一个时间步骤内可能采取的运动范围。窗口通常由线速度(速度的大小)和角速度(速度的方向)两个维度定义。
1. 传统DWA算法应用于感知固定方向的运动，或者说，按照激光雷达或者相机等传感器朝向的方向规划动态窗口法
2. 模型很重要，传统DWA面向的模型有差速模型和全向移动模型，目前工程里面的模型是面向差速模型的
3. 如果感知设备，比如说激光雷达随着车体的旋转而旋转，但是底盘的运动模型是全向移动，DWA算法需要对感知范围等进行改变。举个简单的例子：如果车底盘朝向后方即雷达朝向后方，但是车的运动方向是前方，那么此时就无法对行进方向的障碍物进行判断：![传统DWA局限示意图](传统DWA局限示意图.png)
4. 解决问题3的方案：a.360°二维雷达或者3维雷达 b.雷达安装方向朝向为前，即设置云台让雷达始终朝向运动方向 c.放弃全向移动模型，这里的放弃是指始终让车头方向朝向运动方向，目前来说c实现的成本最低，但是这对全向移动底盘是个浪费
#### 搭建DWA算法需要的几个条件
1. Differential_DWA.launch 文件及相关文件 differential_dwa.cpp 和 differential_dwa.h等是差速底盘的DWA算法搭建，不是全向移动底盘的，需要注意一下
2. 需要局部地图，最好是**局部膨胀地图**，因为算法中没有考虑移动机器人的形状信息
```c++
  local_map_sub = nh.subscribe("/local_map_inflate", 1, &DWAPlanner::local_map_callback, this);
```
3. 需要有一个算法去按照全局路径生成**目标速度**去跟随，该算法也可以在DWA中实现，当然这里我使用的是前端的pure-pursuit算法作为参考速度生成的算法
```c++
target_velocity_sub = nh.subscribe("/velocity_control", 1, &DWAPlanner::target_velocity_callback, this);
```
4. 需要局部终点，也可以直接提供全局终点，但是建议还是按照全局路径去采样提供局部终点
```c++
local_goal_sub = nh.subscribe("/local_goal", 1, &DWAPlanner::local_goal_callback, this);
```
5. 需要里程计数据，因为算法是需要根据当前的运动状态进行采样的，即根据反馈速度采样
```c++
  odom_sub = nh.subscribe("/carto_odom", 1, &DWAPlanner::odom_callback, this);
```
**6.这点网上很多开源算法不会提到，在最开始的阶段没有速度反馈，需要一个激励速度去作为临时采样参照，由此开始车才能动起来，当然这里是临时使用全向移动模型做一个实验，目前这样做是不合理的**
```c++
  // 当前速度过小的时候，提供一个激励
  // 因为是全向移动底盘，计算一个合速度
  double velocity = sqrt(pow(current_velocity.linear.x,2)+pow(current_velocity.linear.y,2));
  if(abs(velocity) <= 0.2)
  {
    velocity = 0.2;
  }
  current_velocity.linear.x = velocity;
```

## 地图处理：
### move_base中的代价地图功能包:
涉及的文件有：**cost_map**，参数配置文件:
  1. costmap_common_params.yaml 关于代价地图的常规的配置参数，以下部分需要注意，如果雷达的话题消息自己进行修改了，sensor_frame 和 topic一定要修改成自己对应的，否则无法构建代价地图
```c++
  #导航包所需要的传感器
  observation_sources: scan
  # 测试carto情况下键入能否使用
  scan: {sensor_frame: laser_link, data_type: LaserScan, topic: scan, marking: true, clearing: true}
```
  2. costmap_local_params.yaml 关于局部地图某些配置参数和常规配置不同，需要修改
  3. global_costmap_params.yaml 全局代价地图中坐标系是否使用静态地图等配置
  4. local_costmap_params.yaml 局部代价地图中坐标系是否使用滚动地图等配置

## 路径跟随算法：
### 初步决定采用PID跟随路径、后续可能会使用MPC跟随控制算法
### Pure-pursuit 路径跟随算法
下面以一张图来展示pure-pursuit算法的原理：![原pure-pursuit解析图](原pure-pursuit解析图.png)
如图所示，为pure-pursuit跟踪算法的示意图。其中，(Cx, Cy)表示当前智能车的位置坐标，(Gx, Gy)表示跟踪轨迹的预瞄点的位置坐标，Ld为预瞄点到车辆后轴的距离即预瞄距离，R表示跟踪的曲率半径。那么根据pure pursuit算法计算出控制量前轮转角δ 以及对应的智能车辆的角速度W。
具体解算如下:![pure-pursuit解算](pure-pursuit解算.png)
改进：因为使用的是全向移动底盘，并且能够原地转向，转弯半径为0，结合移动优势，为了让车运行的更加丝滑，加入五次多项式对需要跟随的轨迹点求取导数，根据导数解算斜率，得出当前状态下世界坐标系下的移动速度，并根据当前车的位置姿态解算出车的局部速度，实现全向移动：解算如下:![五次多项式pure-pursuit跟随](五次多项式pure-pursuit跟随.png)以及![五次多项式pure-pursuit跟随优势分析](五次多项式pure-pursuit跟随优势分析.png)
前端使用astar在静态人工势场地图情况下进行全局规划，使用3阶段Minimum_Snap算法进行优化，最终得出平滑轨迹，但是平滑轨迹的轨迹分辨率大约为0.01m，这导致假如最终优化出来的路径总长度8m，其中将会有800个点，然而我们的定位精度是0.05m也就是5cm，在算法运行过程中车会出现速度突变造成刚性冲击，之后改进的方法是根据地图定位精度对优化的路径进行疏化处理，降低路径点的密度，lf是向前探索距离，ld是固定的向前搜索距离，这里ld取的是定位的分辨率0.05，lf = ld+Vt,V是当前车的反馈速度，t是跟随算法运行一次的时间。车在全向移动的同时对角度进行规划，可以有效的减少车的x y方向上的加速度变化频率以及变化幅度，使车运行更加丝滑。这里我们解算的时时刻刻都是速度上的，对于角速度的规划我们认为默认是要慢于或者等于速度规划的频率的，所以创新性的提出移动速度规划和角速度规划分开进行，因为路径点密度太大，角度规划过于频繁，导致车出现抖动，并且也会影响速度的规划，所以可以根据实际运动情况，设定角度跟随分辨率。是否结合人工势场，目前还没进行测试，但是可以结合人工势场，使车在运行的时候更加安全更加丝滑。
虽然是全向移动底盘，规划角度的意义如下图所是，**非常好理解就是如果不改变车的朝向，对于比较窄的环境，车通过会产生碰撞**，图解如下所示![规划运动方向的优势](规划运动方向的优势.png)。

## 总结：
在对该功能包进行使用的时候养成备份的好习惯，尽量按照代码规范进行变成，所有测试运行节点写到/src/app下面；/src/path_searcher文件夹是全局规划算法；/src/path_optimization文件夹是路径优化算法；/src/path_follow文件加是路径跟随算法或者说轨迹跟踪，二者是一样的

## 环境搭建：
### pcl点云库安装：必须安装，[参考帖子](https://blog.csdn.net/weixin_41836738/article/details/121451965)
1. $ git clone https://github.com/PointCloudLibrary/pcl.git 
2. cd pcl 
3. mkdir release 
4. cd release
5. cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr \ -DBUILD_GPU=ON-DBUILD_apps=ON -DBUILD_examples=ON \ -DCMAKE_INSTALL_PREFIX=/usr .. 
6. make  
7. sudo make install
### ompl库的安装：RRT会用到这个库
1. 安装比较复杂可以参照[ubuntu安装OMPL机器人运动规划库](https://blog.csdn.net/qq_18376583/article/details/127129778)或者其帖子教程也可
**注意：**可能还需要其它的环境这里到时候缺哪个装哪个，实在是记不起来了

## 使用方法：
1. 随便创建一个工作空间，取名work_space
2. 在工作空间创建src文件，将nav_lei功能包复制进去
3. 在work_space工作空间下开启终端，输入catkin_make进行编译
4. 编译通过后终端输入source ./devel/setup.bash
5. 启动节点：roslaunch nav_lei demo_node.launch    目前demo_node.launch中可以同时显示A* JPS RRT三种算法规划的路径 和使用minimum_snap优化的JPS算法路径
如果只对A*仿真 roslaunch nav_lei astar.launch 
只对RRT进行仿真 roslaunch nav_lei rrt.launch
