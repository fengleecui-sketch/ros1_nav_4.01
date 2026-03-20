<!--
 * @Author: your name
 * @Date: 2023-06-26 09:08:31
 * @LastEditTime: 2023-07-17 00:30:32
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/readme.md
 * 可以输入预定的版权声明、个性签名、空行等
-->

# ros1_noetic_nav
nav_sim_control


# 简单说一下各部分功能包的作用
**重点** 在车能够动起来之前，系统在原则上不追求某一方面技术的先进性但要求整体的适应性合理性
## read_laser_data
用来检测雷达状态和读取激光雷达的数据，用于查看激光雷达工作状态，查看当前雷达发出的数据消息
## robot_communication
用来构建各个功能节点之间的消息数据通讯功能包，如果对整个工程进行重新编译，需要先编译该功能包，编译的具体方式可以看该功能包下面的readme
## robot_costmap
用于全局或者局部规划发布地图消息，截止到2023年5月24日还没有合适的局部规划算法和建图算法，该功能包只是发布一个简单的处理过的代价地图，测试各个节点之间接口数据是否正常
## robot_gmapping
用来进行机器人的建图，目前其中有hector和gmapping建图算法，其中gmapping建图是ROS自带的建图算法，构造代价地图可以参照该功能包
## robot_locatization
定位功能包，目前该功能包使用的是pl-icp定位算法，后续可能会部署AMCL定位算法。当然最好是部署谷歌开源的Cartographer定位建图算法，但是因为Carto较高的学习成本和环境配置难度，目前准备先使用pl-icp或者AMCL算法，作为机器人的定位算法
## robot_navigation
导航功能包，其中包括全局规划、局部规划、路径优化、轨迹跟随算法，其中全局规划部署了：A*、JPS、RRT*、Hybrid A*四种算法，局部规划目前只有DWA算法(目前不是很好用)，路径优化算法现在有贝塞尔曲线和Minimum_snap优化算法，轨迹跟随目前只有PID算法
## ros_gazebo_control
用来在仿真环境下观察算法的运行情况，其中包括机器人的xacro文件和三维模型的STL模型文件，可以和ROS自带的定位、导航等功能包进行联合仿真，其中有键盘控制节点以及所需的必要的控制节点，主要用来仿真
## robot_usart
用来和下位机进行实时通讯，获取轮速计、陀螺仪数据消息，用来和上位机进行数据融合并给下位机发送控制指令
## cartographer_ros
谷歌开源SLAM算法，2D激光雷达定位天花板，目前已经初步部署成功，在使用之前请阅读该功能包下面的**readme_carto_install.md**文件

# **如何编译**
**在引入Cartographer之后，整个工程的编译发生了一些变化，这里简单说一下，在配置完成Carto的环境要求的情况下(具体可以看readme_carto_install.md)，需要遵循以下编译的顺序**
**1.使用catkin_make  --pkg robot_communication先对robot_communication进行编译**
**2.使用catkin_make对于除了cartographer_ros之外的功能包进行编译，编译之后会生成devel和build文件**
**3.使用./catkin_make.sh对于cartographer_ros及其他功能包编译，这里在第二部编译成功的情况下，可以正常编译成功，编译之后会生成build_isolated，devel_isolated,install_isolated文件**
**4.后来发现两个包一同编译会报错，需要分开进行编译，体现的结果是cartographer_ros可以正常编译和使用，但是自己的功能包无法进行源代码的修改之后编译再运行**
**注意：cartographer目前已经是2D定位天花板，这里基本上不会对代码进行任何修改，所以我们要做的基本上就是调参，但是每次调参之后都需要调用./catkin_make.sh进行编译，否则参数不会生效**

# .sh文件的使用
**注意：.sh文件中有<gnome-terminal -t "roskill" -x bash -c "rosnode kill --all">这句话的时候不要连续开启多个.sh文件，这句话的含义是杀死所有ros的节点，会导致后启动的节点和当前运行的节点运行出现问题**
## catkin_make.sh
用来编译cartographer_ros功能包
## laser_sim_feature.sh
启动gazebo仿真中的雷达和小车以及世界情况下的仿真，并且启动了特征点匹配，在运行之前记得更改**read_laser_data功能包下面 launch文件中的feature_detection文件中的参数，如何修改launch文件中已经给出**
## laser_link_test.sh
是用来测试实际雷达和真实雷达是否链接成功的，只是单独测试雷达连接是否成功
## laser_acl_feature.sh
启动真正的雷达特征点匹配仿真，在运行之前记得更改**read_laser_data功能包下面 launch文件中的feature_detection文件中的参数，如何修改launch文件中已经给出**
## laser_change_param.sh
这里对雷达的参数进行了修改，包括扫描的方向，扫描的范围(实际使用的过程中需要注意，因为机械结构安装的原因，可能需要重新设置盲区)
## gmapping_test.sh
用来建立地图的测试文件
## nav_sim.sh
使用gazebo发布的理想状态下真实数值的里程计进行定位用来作运动规划算法的上位机仿真测试
## pl_icp_sim.sh
使用pl_icp算法在仿真状态下的算法测试启动文件
## usart_test.sh
测试读取下位机发送的传感器数据
