# 该功能包的作用是定位
## 目前代码来源是CSDN上一位SLAM研究方向的博主[零基础如何入门激光SLAM](https://blog.csdn.net/tiancailx/article/details/111358640)
**重点**
1. 介绍了如何学习激光SLAM并且开源了很多定位算法icp、pl-icp就是其中两种，并且开源了目前比较常用的二维场景下的建图算法hector建图算法和ros自带的gmapping建图算法
2. 提取激光雷达特征点、icp、pl-icp、建图等代码例程[开源代码链接](https://github.com/xiangli0608/Creating-2D-laser-slam-from-scratch)

## 介绍一下下面几个launch文件的作用：
## actual_world.launch
按照小车实际场景中的运动配置的，这里面包括pl-icp定位和truth pose即gazebo输出的理想状态下的里程计信息
**重点:**使用gazebo输出的理想状态下的里程计信息，这里的里程计输出的世界坐标系下的位置和速度矢量，注意是世界坐标系下的位置和速度矢量以及加速度矢量，所以说无需记性坐标变换
## actual_world.launch
模拟情况下按照实际情况下提供传感器信息
## carto_locatization.launch
把carto定位消息转换成实际导航的需要的定位消息
## chassis_control.launch
用来将世界坐标系下的速度消息转换为车底盘的局部坐标系下的消息
## odom_calibra_pcl.launch
之前写的准备对pcl点云滤波的算法，但是不用了
## plicp_odometry_sim.launch
启动的是仿真状态下pl-icp定位算法，雷达和里程计使用的仿真状态下的
## plicp_odometry_acl.launch
启动的是实际状态下pl-icp定位算法，雷达和里程计使用的是真正的
## truth_odometry.launch
gazebo发布的真值里程计的启动文件，用于仿真

### 

## 总结：


## 环境搭建：
### ros的tf2-sensor安装
sudo apt-get install ros-noetic-tf2-sensor-msgs
### csm库的安装：
sudo apt-get install ros-noetic-csm
**注意：**可能还需要其它的环境这里到时候缺哪个装哪个，实在是记不起来了

## 使用方法：
1. 随便创建一个工作空间，取名work_space
2. 在工作空间创建src文件，将nav_lei功能包复制进去
3. 在work_space工作空间下开启终端，输入catkin_make进行编译
4. 编译通过后终端输入source ./devel/setup.bash
5. 启动节点：
