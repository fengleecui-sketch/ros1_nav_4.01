#ifndef ROBOT_USART_USART_CONFIG_H
#define ROBOT_USART_USART_CONFIG_H

// ROS 相关头文件
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

// Boost.Asio 串口通信头文件
#include <boost/asio.hpp>

// C++ 标准库头文件
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

// 自定义消息头文件
#include "robot_communication/localizationInfoBroadcast.h"
#include "robot_communication/sensorData.h"
#include "robot_communication/chassisControl.h"

// JSON 配置文件读取
#include "Jason/jason.hpp"

// CRC 校验头文件
#include "robot_usart/crc.h"

// 终端彩色字体头文件
#include "robot_usart/fontColor.h"

// 为 json 定义别名，便于后续使用
using json = nlohmann::json;

// 弧度转角度系数
#define RAD_TO_ANGLE 57.2957795f

// 串口接收帧长度：19字节
#define RX_LENGTH 19

// 串口发送帧长度：14字节
#define TX_LENGTH 14

/**
 * @brief 接收到的底盘传感器数据结构体
 * 
 * 该结构体用于保存从下位机串口接收到并解码后的底盘状态信息。
 */
typedef struct SensorData
{
    // 底盘x方向线速度，单位 m/s
    float chassix_x_linear_velocity;

    // 底盘y方向线速度，单位 m/s
    float chassis_y_linear_velocity;

    // 底盘x方向加速度，单位 m/s^2
    float chassis_x_accelerate;

    // 底盘y方向加速度，单位 m/s^2
    float chassis_y_accelerate;

    // 底盘偏航角，单位 deg（解码后原始角度值）
    float chassis_yaw;

    // 下位机发送上来的时间戳，单位由协议决定
    uint32_t time_stamp_10us;
} SensorData_Define;

/**
 * @brief 串口通信配置与收发主类
 * 
 * 该类主要负责：
 * 1. 串口初始化与配置
 * 2. 串口发送控制命令
 * 3. 串口接收下位机传感器数据
 * 4. 对接收数据进行解码
 * 5. 将解码后的数据发布为 ROS 话题
 */
class usartConfig
{
public:
    /**
     * @brief 构造函数
     * 初始化接收数据、串口、话题、定时器、线程等
     */
    usartConfig();

    /**
     * @brief 析构函数
     * 程序结束时关闭串口资源
     */
    ~usartConfig();

    /**
     * @brief 初始化接收数据结构体
     */
    void recData_Init(void);

    /**
     * @brief 串口及ROS相关配置初始化
     */
    void Usart_Config(void);

    /**
     * @brief 定时发送控制命令回调函数
     * @param  ros定时器事件
     */
    void controlCmdSendCallback(const ros::TimerEvent &);

    /**
     * @brief 读取一帧串口数据
     * @return int
     * >0 ：接收成功
     * -1 ：CRC校验失败
     * -2 ：数据长度错误
     * -3 ：读取异常
     * -4 ：串口离线
     * -5 ：帧头错误
     * -6 ：类型字节错误
     */
    int ReadUsart();

    /**
     * @brief 串口接收线程函数
     * 后台持续读取串口数据
     */
    void RecvThread();

    /**
     * @brief 关闭串口
     */
    void UsartClose();

    /**
     * @brief 重启串口
     */
    void UsartRestart();

    /**
     * @brief 显示串口接收频率
     * 主要用于调试
     */
    void displayUsartFreq(void);

private:
    /**
     * @brief 对接收到的原始串口数据进行解码
     */
    void recData_Decode(void);

    /**
     * @brief 发布底盘传感器消息定时器回调
     * @param ros定时器事件
     */
    void PubSensor_DataSendCallback(const ros::TimerEvent &);

    /**
     * @brief 全局速度控制话题回调函数
     * @param msg 接收到的底盘控制消息
     */
    void GlobalVelSubCallback(const robot_communication::chassisControlConstPtr &msg);

    /**
     * @brief /cmd_vel_auto 话题回调函数
     * @param msg geometry_msgs::Twist 消息
     */
    void CmdVelCallback(const geometry_msgs::TwistConstPtr &msg);

    /**
     * @brief 打开串口
     * @param port_name 串口设备名称，例如 /dev/ttyUSB0
     * @param baud_rate 波特率
     * @return true 打开成功
     * @return false 打开失败
     */
    bool OpenSerial(const std::string &port_name, int baud_rate);

    /**
     * @brief 从参数服务器读取串口参数
     */
    void ConfigSerialPort();

    /**
     * @brief 以16进制形式打印接收到的原始数据
     * @param data 数据首地址
     * @param len 数据长度
     */
    void PrintHexBuffer(const uint8_t *data, int len);

private:
    // 私有节点句柄，用于读取私有参数、创建订阅器、发布器、定时器
    ros::NodeHandle private_node_;

    // 存放解码后的串口接收数据
    SensorData_Define recSensor_;

    // Boost.Asio 的 io_service，对串口进行底层管理
    boost::asio::io_service io_service_;

    // Boost.Asio 串口对象，使用智能指针自动管理生命周期
    std::unique_ptr<boost::asio::serial_port> serial_port_;

    // 定时发送控制命令的定时器
    ros::Timer cmdTimer_;

    // 定时发布底盘传感器消息的定时器
    ros::Timer sensorTimer_;

    // 底盘传感器数据发布器
    ros::Publisher SensorDataPub_;

    // 里程计发布器（当前版本中尚未使用）
    ros::Publisher aclOdomDataPub_;

    // 全局速度控制消息订阅器
    ros::Subscriber globalVelSub_;

    // 自动速度控制消息订阅器
    ros::Subscriber cmdVelSub_;

    // 要发布的底盘传感器 ROS 消息
    robot_communication::sensorData chassiSensor_;

    // 里程计消息对象（当前版本中尚未使用）
    nav_msgs::Odometry aclOdom_;

    // 当前控制命令缓存
    robot_communication::chassisControl controlMotion_;

    // 串口发送缓存数组
    uint8_t usartTxBuffer_[TX_LENGTH];

    // CRC 校验对象
    CRC usart_check_;

    // 是否使用全局速度控制模式
    bool use_global_ = false;

    // 是否更新过速度控制量，当前版本中保留变量
    bool velocity_update_ = false;

    // 串口接收频率计数器
    uint32_t freq_ = 0;

    // 串口接收频率统计使能标志
    uint8_t freqFlag_ = 0;

    // 线程锁，保护接收线程中的共享资源
    std::mutex thread_locker_;

    // 串口接收线程对象
    std::thread recv_thread_;

    // 接收线程运行状态标志
    std::atomic<bool> on_running_{false};

    // 接收帧固定帧头字节
    static const int frame_header_ = 0x05;

    // 接收帧固定类型字节
    static const int frame_type_   = 0xFA;

    // 串口最大缓存长度
    static const int UART_XMIT_SIZE = 4096;

    // 串口设备默认名称
    std::string port_name_ = "/dev/ttyUSB0";

    // 串口默认波特率
    int baud_rate_ = 460800;

    // JSON 参数对象，当前版本中预留
    json param_;

    /**
     * @brief 串口接收状态结构体
     * 
     * 用于实现接收状态机，管理串口接收过程中的：
     * 1. 当前已接收长度
     * 2. 是否在接收帧头
     * 3. 是否在接收类型字节
     * 4. 是否在接收数据区
     * 5. 是否检测到错误帧
     */
    struct RecvStatus
    {
        // 当前已经接收到的字节数
        int recv_len = 0;

        // 是否处于接收帧头状态
        bool ON_RECV_HEADER = true;

        // 是否处于接收类型字节状态
        bool ON_RECV_TYPE = true;

        // 是否处于接收数据区状态
        bool ON_RECV_DATA = true;

        // 错误帧标志
        bool WRONG_TICK = false;

        // 接收缓存区
        unsigned char buff[UART_XMIT_SIZE] = {0};

        /**
         * @brief 将接收状态机复位
         */
        void reset()
        {
            recv_len = 0;
            ON_RECV_HEADER = true;
            ON_RECV_TYPE = true;
            ON_RECV_DATA = true;
            WRONG_TICK = false;
        }

        /**
         * @brief 当检测到错误帧时，设置错误标志并复位接收状态
         */
        void set_wrong_tick()
        {
            WRONG_TICK = true;
            ON_RECV_HEADER = true;
            ON_RECV_TYPE = true;
            ON_RECV_DATA = true;
            recv_len = 0;
        }
    } recv_buff_;
};

#endif