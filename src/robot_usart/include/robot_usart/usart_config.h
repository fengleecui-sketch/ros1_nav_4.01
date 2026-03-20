/*
 * @Author: your name
 * @Date: 2023-06-26 09:08:31
 * @LastEditTime: 2025-04-17 11:42:49
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_usart/include/robot_usart/usart_config.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __USART_CONFIG_H
#define __USART_CONFIG_H

//--ros库函数
#include "ros/ros.h"
#include <serial/serial.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include "iostream"
// 针对舵轮王新增的头文件 -- wrc2026_01_19
#include <geometry_msgs/Twist.h>

// tf2
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"

// 包含自定义消息包
#include "robot_communication/localizationInfoBroadcast.h"
#include "robot_communication/sensorData.h"
#include "robot_communication/chassisControl.h"
//--c++库函数
#include "bits/stdc++.h"
using namespace std;
#include <sys/timeb.h>
//参数读取
#include "Jason/jason.hpp"
using json = nlohmann::json;


#include "fontColor.h"
#include "crc.h"

#include <boost/asio.hpp>
using namespace boost::asio;

#define RAD_TO_ANGLE 57.2957795f

// 缓冲区设定大一点，因为在打开别的节点的时候可能导致会给串口冲炸，多缓存一些再清空
#define RX_LENGTH   19   //接收数据长度
#define TX_LENGTH   14   //发送数据长度

// 接收到的均为当前车的局部数据
typedef struct SensorData{
  float  chassix_x_linear_velocity;     //底盘x轴方向上的线速度
  float  chassis_y_linear_velocity;     //底盘y轴方向上的线速度

  float  chassis_x_accelerate;          //底盘x轴方向上的加速度
  float  chassis_y_accelerate;          //底盘y轴方向上的加速度

  float  chassis_yaw;                   //底盘偏航角

  uint32_t time_stamp_10us;    //下位机发送的时间戳
}SensorData_Define;

class usartConfig
{
  public:
    usartConfig();
    ~usartConfig();

    void recData_Init(void);
    // 串口配置
    void Usart_Config(void);
    // 串口发送回调函数
    void controlCmdSendCallback(const ros::TimerEvent &);
    // 发送控制指令
    void controlCMdSend(void);
    // 读取串口数据
    int ReadUsart();
    // 串口接收线程
    void RecvThread();
    //  串口关闭
    void UsartClose();
    // 串口重启
    void UsartRestart();
    // 执行
    void run();
    // 显示串口接收频率和正确率
    void displayUsartFreq(void);
  private:
    SensorData_Define recSensor;
    /* data */ 
    boost::asio::serial_port *serial_port_;            // 设备句柄
    
    //下位机控制指令定时器
    ros::Timer cmdTimer;
    // 发布传感器消息
    ros::Timer sensorTimer;
    // --串口发送数组
    uint8_t usartTxBuffer[TX_LENGTH];
    // CRC校验
    CRC usart_check;

    // 是否使用全局速度
    bool use_global;

    // 读取串口的频率
    uint32_t freq;
    // 是否读取频率标志位
    uint8_t freqFlag;

    std::mutex thread_locker;

    bool on_running = false;

    static const int frame_header = 0x05;

    static const int UART_XMIT_SIZE =
        4096;  // Linux内核中指定的串口缓冲区大小，如果想加大可以修改对应代码重新编译内核

    //jason参数
    json param;
    struct RecvStatus {
      int recv_len = 0;
      bool ON_RECV_HEADER = true;
      bool ON_RECV_TYPE = true;
      bool ON_RECV_DATA = true;
      bool WRONG_TICK = false;
      unsigned char buff[UART_XMIT_SIZE];
      void reset() {
        recv_len = 0;
        ON_RECV_HEADER = true;
        ON_RECV_TYPE = true;
        ON_RECV_DATA = true;
      }
      void set_wrong_tick() {
        WRONG_TICK = true;
        ON_RECV_HEADER = true;
        ON_RECV_TYPE = true;
        ON_RECV_DATA = true;
      }
    } recv_buff;

    // 串口接收数据解码
    void recData_Decode(void);
    // 发布传感器消息
    void PubSensor_DataSendCallback(const ros::TimerEvent &);


    ros::Publisher SensorDataPub;     //底盘传感器消息话题发布
    robot_communication::sensorData chassiSensor;  //发布底盘传感器消息

    // 话题消息发布
    ros::Publisher aclOdomDataPub;
    nav_msgs::Odometry aclOdom;        //里程计数据 这里发布的是全局的传感器数据
    // 底盘数据话题订阅
    ros::Subscriber globalVelSub;      // 全局速度订阅
    // 全局速度数据
    robot_communication::chassisControl controlMotion;      //全局运动数据
    bool velocity_update;
    // 全局速度订阅回调函数
    void GlobalVelSubCallback(const robot_communication::chassisControlConstPtr &msg);
    // 真实机器人订阅/cmd_vel
    ros::Subscriber cmdVelSub;
    void CmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
    
  private:
    ros::NodeHandle private_node;  // ros中的私有句柄
};


#endif //
