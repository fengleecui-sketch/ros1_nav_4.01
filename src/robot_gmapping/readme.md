<!--
 * @Author: your name
 * @Date: 2023-06-14 17:33:36
 * @LastEditTime: 2023-06-25 21:20:48
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_gmapping/readme.md
 * 可以输入预定的版权声明、个性签名、空行等
-->
# 该功能包目前主要用于建立出局部地图，建立的局部地图经过robot_costmap功能包处理之后发布话题用来进行局部规划
## 目前代码来源是CSDN上一位SLAM研究方向的博主[零基础如何入门激光SLAM](https://blog.csdn.net/tiancailx/article/details/111358640)
**重点**
1. 介绍了如何学习激光SLAM并且开源了很多定位算法icp、pl-icp就是其中两种，并且开源了目前比较常用的二维场景下的建图算法hector建图算法和ros自带的gmapping建图算法
2. 提取激光雷达特征点、icp、pl-icp、建图等代码例程[开源代码链接](https://github.com/xiangli0608/Creating-2D-laser-slam-from-scratch)
3. 目前主要使用**hector_mapping**算法和gmapping(ros自带的以代码形式呈现出来)算法
4. occupancy_grid是呈现出不同数值下的地图效果图
## 目前使用主要是hector_mapping_node节点使用hector进行建图

### 

## 总结：


## 环境搭建：
### osqp库的安装：[参考链接](https://blog.csdn.net/whuzhang16/article/details/111508384)
1. git clone --recursive https://github.com/oxfordcontrol/osqp
2. cd osqp
3. mkdir build
4. cd build
5. cmake .. -DBUILD_SHARED_LIBS=ON
6. make -j6
7. sudo make install
### osqp-eigen库的安装：[参考链接](https://blog.csdn.net/qjj18776858511/article/details/125963379)
1. 打开终端，进入osqp-eigen文件夹，然后执行以下指令
2. mkdir build       
3. cd build          
4. cmake .. -DCMAKE_INSTALL_PREFIT=usr/local/osqp-eigen
5. sudo make install
6. source ~/.bashrc  
### csm库的安装：
sudo apt-get install ros-kinetic-csm
**注意：**可能还需要其它的环境这里到时候缺哪个装哪个，实在是记不起来了

## 使用方法：
1. 随便创建一个工作空间，取名work_space
2. 在工作空间创建src文件，将nav_lei功能包复制进去
3. 在work_space工作空间下开启终端，输入catkin_make进行编译
4. 编译通过后终端输入source ./devel/setup.bash
5. 启动节点：

