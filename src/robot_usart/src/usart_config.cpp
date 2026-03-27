#include "robot_usart/usart_config.h"

#include <unistd.h>
#include <iomanip>
#include <sstream>

/**
 * @brief usartConfig 类构造函数
 * 1. 初始化接收数据结构体
 * 2. 初始化串口、话题、定时器、接收线程
 */
usartConfig::usartConfig() : private_node_("~")
{
    recData_Init();   // 初始化接收数据变量
    Usart_Config();   // 初始化串口和ROS相关配置
}

/**
 * @brief usartConfig 类析构函数
 * 程序退出时关闭串口
 */
usartConfig::~usartConfig()
{
    UsartClose();
}

/**
 * @brief 打开串口并配置串口参数
 * @param port_name 串口设备名称，例如 /dev/ttyUSB0
 * @param baud_rate 波特率，例如 460800
 * @return true 打开成功
 * @return false 打开失败
 */
bool usartConfig::OpenSerial(const std::string &port_name, int baud_rate)
{
    try
    {
        // 创建串口对象
        serial_port_.reset(new boost::asio::serial_port(io_service_));

        // 打开指定串口
        serial_port_->open(port_name);

        // 设置波特率
        serial_port_->set_option(boost::asio::serial_port::baud_rate(baud_rate));

        // 设置流控：无流控
        serial_port_->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));

        // 设置校验位：无校验
        serial_port_->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));

        // 设置停止位：1位停止位
        serial_port_->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));

        // 设置数据位：8位
        serial_port_->set_option(boost::asio::serial_port::character_size(8));

        ROS_INFO("Serial open success: %s, baud=%d", port_name.c_str(), baud_rate);
        return true;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Open serial failed: %s", e.what());

        // 打开失败时释放串口对象
        serial_port_.reset();
        return false;
    }
}

/**
 * @brief 从ROS参数服务器读取串口相关参数
 * 包括：
 * 1. 串口号
 * 2. 波特率
 * 3. 是否使用全局速度控制
 */
void usartConfig::ConfigSerialPort()
{
    // 读取串口设备名，默认 /dev/ttyUSB0
    private_node_.param<std::string>("port_name", port_name_, std::string("/dev/ttyUSB0"));

    // 读取波特率，默认 460800
    private_node_.param<int>("baud_rate", baud_rate_, 460800);

    // 读取是否使用全局速度，默认 false
    private_node_.param<bool>("use_global", use_global_, false);

    ROS_INFO("port_name: %s", port_name_.c_str());
    ROS_INFO("baud_rate: %d", baud_rate_);
    ROS_INFO("use_global: %d", static_cast<int>(use_global_));
}

/**
 * @brief 串口功能整体初始化函数
 * 主要完成：
 * 1. 读取参数
 * 2. 打开串口
 * 3. 初始化话题订阅和发布
 * 4. 初始化定时器
 * 5. 启动串口接收线程
 */
void usartConfig::Usart_Config(void)
{
    // 读取参数
    ConfigSerialPort();

    // 打开串口
    if (!OpenSerial(port_name_, baud_rate_))
    {
        ROS_ERROR("Initial serial open failed.");
    }

    // 订阅底盘全局速度控制话题
    globalVelSub_ = private_node_.subscribe("/acl_velocity", 10, &usartConfig::GlobalVelSubCallback, this);

    // 订阅自动控制速度话题
    cmdVelSub_ = private_node_.subscribe("/cmd_vel_auto", 10, &usartConfig::CmdVelCallback, this);

    // 50Hz定时发送控制命令给下位机
    cmdTimer_ = private_node_.createTimer(ros::Duration(0.02), &usartConfig::controlCmdSendCallback, this);

    // 200Hz定时发布底盘传感器数据
    sensorTimer_ = private_node_.createTimer(ros::Duration(0.005), &usartConfig::PubSensor_DataSendCallback, this);

    // 发布底盘传感器数据话题
    SensorDataPub_ = private_node_.advertise<robot_communication::sensorData>("/chassis_sensor_data", 1);

    // 如果接收线程还未启动，则启动接收线程
    if (!on_running_)
    {
        on_running_ = true;
        recv_thread_ = std::thread(&usartConfig::RecvThread, this);
        recv_thread_.detach();   // 分离线程，让其在后台运行
    }
}

/**
 * @brief 关闭串口
 * 1. 停止接收线程运行标志
 * 2. 关闭串口设备
 */
void usartConfig::UsartClose()
{
    on_running_ = false;

    try
    {
        if (serial_port_ && serial_port_->is_open())
        {
            serial_port_->close();
            ROS_INFO("Serial closed.");
        }
    }
    catch (const std::exception &e)
    {
        ROS_WARN("Serial close exception: %s", e.what());
    }
}

/**
 * @brief 串口重启函数
 * 在串口离线、读取异常时调用：
 * 1. 关闭串口
 * 2. 释放串口对象
 * 3. 延时一段时间
 * 4. 重新打开串口
 */
void usartConfig::UsartRestart()
{
    try
    {
        if (serial_port_ && serial_port_->is_open())
        {
            serial_port_->close();
        }
    }
    catch (...)
    {
        // 这里即使关闭异常，也不影响后续重启流程
    }

    // 释放原串口对象
    serial_port_.reset();

    // 延时100ms，避免立即重连失败
    usleep(1000 * 100);

    // 重新打开串口
    OpenSerial(port_name_, baud_rate_);
}

/**
 * @brief 定时发送控制命令给下位机
 * 发送帧格式：
 * [0]   0x03
 * [1]   0xFC
 * [2:3] x速度
 * [4:5] y速度
 * [6:7] 底盘角度
 * [8:10] 底盘角速度
 * [11]  use_global标志
 * [12:13] CRC16
 */
void usartConfig::controlCmdSendCallback(const ros::TimerEvent &)
{
    try
    {
        // 串口未打开则直接返回
        if (!(serial_port_ && serial_port_->is_open()))
        {
            return;
        }

        // 帧头
        usartTxBuffer_[0] = 0x03;
        usartTxBuffer_[1] = 0xFC;

        // 将浮点控制量编码为整型发送
        int x_speed = static_cast<int>((controlMotion_.xSpeed + 4.000f) * 1000.0f);
        int y_speed = static_cast<int>((controlMotion_.ySpeed + 4.000f) * 1000.0f);
        int yaw_deg  = static_cast<int>((controlMotion_.chassisAngle * RAD_TO_ANGLE + 180.0f) * 100.0f);
        int gyro_deg = static_cast<int>((controlMotion_.chassisGyro  * RAD_TO_ANGLE + 360.0f) * 100.0f);

        // x速度高字节、低字节
        usartTxBuffer_[2] = (x_speed >> 8) & 0xFF;
        usartTxBuffer_[3] = x_speed & 0xFF;

        // y速度高字节、低字节
        usartTxBuffer_[4] = (y_speed >> 8) & 0xFF;
        usartTxBuffer_[5] = y_speed & 0xFF;

        // yaw角高字节、低字节
        usartTxBuffer_[6] = (yaw_deg >> 8) & 0xFF;
        usartTxBuffer_[7] = yaw_deg & 0xFF;

        // 角速度共3字节，高位在前
        usartTxBuffer_[8]  = (gyro_deg >> 16) & 0xFF;
        usartTxBuffer_[9]  = (gyro_deg >> 8) & 0xFF;
        usartTxBuffer_[10] = gyro_deg & 0xFF;

        // 是否使用全局速度控制
        usartTxBuffer_[11] = static_cast<uint8_t>(use_global_ ? 1 : 0);

        // 追加CRC16校验
        usart_check_.Append_CRC16_Check_Sum(usartTxBuffer_, TX_LENGTH);

        // 将整帧数据写入串口
        boost::asio::write(*serial_port_, boost::asio::buffer(usartTxBuffer_, TX_LENGTH));
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Serial write failed: %s", e.what());
    }
}

/**
 * @brief 将接收到的字节数组以16进制打印出来
 * @param data 数据首地址
 * @param len  数据长度
 */
void usartConfig::PrintHexBuffer(const uint8_t *data, int len)
{
    std::ostringstream oss;
    for (int i = 0; i < len; ++i)
    {
        oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(data[i]) << " ";
    }

    // 打印接收到的原始16进制数据
    // ROS_INFO_STREAM("RX HEX: " << oss.str());
}

/**
 * @brief 从串口读取一帧完整数据
 * 接收流程：
 * 1. 先读帧头
 * 2. 再读类型字节
 * 3. 再读剩余数据直到19字节完整
 * 4. 进行CRC校验
 * 5. 校验通过则解码
 *
 * @return int
 * >0  : 成功，返回数据类型
 * -1  : CRC错误
 * -2  : 长度错误
 * -3  : 读取异常
 * -4  : 串口未打开
 * -5  : 帧头错误
 * -6  : 类型错误
 */
int usartConfig::ReadUsart()
{
    // 串口未打开
    if (!(serial_port_ && serial_port_->is_open()))
    {
        return -4;
    }

    try
    {
        // -----------------------------
        // 第一步：接收帧头
        // -----------------------------
        if (recv_buff_.ON_RECV_HEADER)
        {
            // 如果当前缓存中还没有字节，则先读取1字节
            if (recv_buff_.recv_len < 1)
            {
                int recv_len = boost::asio::read(*serial_port_, boost::asio::buffer(recv_buff_.buff, 1));
                if (recv_len <= 0) return -3;
                recv_buff_.recv_len = recv_len;
            }

            // 判断帧头是否正确
            if (recv_buff_.buff[0] == frame_header_ && (!recv_buff_.WRONG_TICK))
            {
                recv_buff_.ON_RECV_HEADER = false;
            }
            else
            {
                // 帧头错误，复位状态机
                recv_buff_.WRONG_TICK = false;
                recv_buff_.reset();
                return -5;
            }
        }

        // -----------------------------
        // 第二步：接收类型字节
        // -----------------------------
        if (recv_buff_.ON_RECV_TYPE)
        {
            // 如果当前长度不足2字节，则继续再读1字节
            if (recv_buff_.recv_len < 2)
            {
                int recv_len = boost::asio::read(*serial_port_, boost::asio::buffer(recv_buff_.buff + 1, 1));
                if (recv_len <= 0) return -3;
                recv_buff_.recv_len += recv_len;
            }

            // 判断类型字节是否正确
            if (recv_buff_.buff[1] == frame_type_)
            {
                recv_buff_.ON_RECV_TYPE = false;
            }
            else
            {
                // 类型字节错误，复位状态机
                recv_buff_.set_wrong_tick();
                return -6;
            }
        }

        // -----------------------------
        // 第三步：接收整帧剩余数据
        // -----------------------------
        if (recv_buff_.ON_RECV_DATA)
        {
            int expected_recv_len = RX_LENGTH;

            // 如果当前接收长度不足一帧，则继续读取剩余字节
            if (recv_buff_.recv_len < expected_recv_len)
            {
                int recv_len = boost::asio::read(
                    *serial_port_,
                    boost::asio::buffer(recv_buff_.buff + recv_buff_.recv_len,
                                        expected_recv_len - recv_buff_.recv_len));
                if (recv_len <= 0) return -3;
                recv_buff_.recv_len += recv_len;
            }

            // 如果长度仍然不够，返回长度错误
            if (recv_buff_.recv_len < expected_recv_len)
            {
                return -2;
            }

            // CRC校验通过
            if (usart_check_.Verify_CRC16_Check_Sum(recv_buff_.buff, expected_recv_len))
            {
                // 打印接收到的整帧16进制数据
                PrintHexBuffer(recv_buff_.buff, expected_recv_len);

                // 对接收数据进行解码
                recData_Decode();

                // 如果开启频率统计，则计数加1
                if (freqFlag_ == 1)
                {
                    freq_++;
                }

                recv_buff_.ON_RECV_DATA = false;
            }
            else
            {
                // CRC校验失败
                ROS_WARN("CRC check failed.");
                PrintHexBuffer(recv_buff_.buff, expected_recv_len);
                recv_buff_.set_wrong_tick();
                return -1;
            }
        }

        // 返回类型字节
        return recv_buff_.buff[1];
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("ReadUsart exception: %s", e.what());
        return -3;
    }
}

/**
 * @brief 串口接收线程函数
 * 线程循环执行：
 * 1. 调用 ReadUsart() 读取串口数据
 * 2. 根据返回值判断是否成功、是否错误、是否需要重启串口
 */
void usartConfig::RecvThread()
{
    ROS_INFO("RecvThread started.");

    // 线程持续运行，直到节点退出或 on_running_ 被置为 false
    while (on_running_ && ros::ok())
    {
        int ret = 0;

        // 加锁，避免串口读和其他操作冲突
        {
            std::lock_guard<std::mutex> lock(thread_locker_);
            ret = ReadUsart();
        }

        // 成功接收到一帧数据
        if (ret > 0)
        {
            recv_buff_.reset();
        }
        // CRC错误
        else if (ret == -1)
        {
            ROS_WARN_THROTTLE(1.0, "CRC error.");
        }
        // 长度错误
        else if (ret == -2)
        {
            ROS_WARN_THROTTLE(1.0, "Read length error.");
        }
        // 串口读取异常
        else if (ret == -3)
        {
            ROS_WARN_THROTTLE(1.0, "Serial read error, restart serial.");
            UsartRestart();
            usleep(1000 * 100);
        }
        // 串口离线
        else if (ret == -4)
        {
            ROS_WARN_THROTTLE(1.0, "Serial offline, restart serial.");
            UsartRestart();
            usleep(1000 * 100);
        }
        // 帧头不匹配
        else if (ret == -5)
        {
            ROS_DEBUG("Header mismatch.");
        }
        // 类型字节不匹配
        else if (ret == -6)
        {
            ROS_DEBUG("Type mismatch.");
        }
    }

    ROS_INFO("RecvThread exit.");
}

/**
 * @brief 初始化接收数据结构体
 * 程序启动时将所有接收量清零
 */
void usartConfig::recData_Init(void)
{
    recSensor_.chassix_x_linear_velocity = 0.0f;
    recSensor_.chassis_y_linear_velocity = 0.0f;
    recSensor_.chassis_x_accelerate = 0.0f;
    recSensor_.chassis_y_accelerate = 0.0f;
    recSensor_.chassis_yaw = 0.0f;
    recSensor_.time_stamp_10us = 0;
}

/**
 * @brief 对接收到的串口数据进行解码
 * 接收帧格式中：
 * [2:3]   x线速度
 * [4:5]   y线速度
 * [6:7]   x加速度
 * [8:9]   y加速度
 * [10:11] yaw角
 * [12:15] 时间戳
 */
void usartConfig::recData_Decode(void)
{
    // 解码x方向线速度
    recSensor_.chassix_x_linear_velocity =
        ((recv_buff_.buff[2] << 8 | recv_buff_.buff[3]) - 4000.0f) / 1000.0f;

    // 解码y方向线速度
    recSensor_.chassis_y_linear_velocity =
        ((recv_buff_.buff[4] << 8 | recv_buff_.buff[5]) - 4000.0f) / 1000.0f;

    // 解码x方向加速度
    recSensor_.chassis_x_accelerate =
        ((recv_buff_.buff[6] << 8 | recv_buff_.buff[7]) - 5000.0f) / 1000.0f;

    // 解码y方向加速度
    recSensor_.chassis_y_accelerate =
        ((recv_buff_.buff[8] << 8 | recv_buff_.buff[9]) - 5000.0f) / 1000.0f;

    // 解码yaw角（单位：度）
    recSensor_.chassis_yaw =
        ((recv_buff_.buff[10] << 8 | recv_buff_.buff[11]) - 18000.0f) / 100.0f;

    // 解析4字节时间戳，[12][13][14][15]
    recSensor_.time_stamp_10us =
        (static_cast<uint32_t>(recv_buff_.buff[12]) << 24) |
        (static_cast<uint32_t>(recv_buff_.buff[13]) << 16) |
        (static_cast<uint32_t>(recv_buff_.buff[14]) << 8)  |
        (static_cast<uint32_t>(recv_buff_.buff[15]));
}

/**
 * @brief 定时发布底盘传感器消息
 * 将 recSensor_ 中的解析结果转换为 ROS 消息后发布
 */
void usartConfig::PubSensor_DataSendCallback(const ros::TimerEvent &)
{
    // 将内部解析数据写入ROS消息
    chassiSensor_.local_x_Veloc = recSensor_.chassix_x_linear_velocity;
    chassiSensor_.local_y_Veloc = recSensor_.chassis_y_linear_velocity;

    chassiSensor_.local_x_Accel = recSensor_.chassis_x_accelerate;
    chassiSensor_.local_y_Accel = recSensor_.chassis_y_accelerate;

    // yaw 转成弧度发布
    chassiSensor_.yaw = recSensor_.chassis_yaw / 57.2957795f;

    // 时间戳直接赋值
    chassiSensor_.timeStamp_10us = recSensor_.time_stamp_10us;

    // 发布话题
    SensorDataPub_.publish(chassiSensor_);

    // 每隔1秒打印一次当前发布的数据，便于调试
    ROS_INFO_THROTTLE(1.0,
                  "pub sensor: vx=%.3f vy=%.3f ax=%.3f ay=%.3f yaw=%.3f ts=%u",
                  chassiSensor_.local_x_Veloc,
                  chassiSensor_.local_y_Veloc,
                  chassiSensor_.local_x_Accel,
                  chassiSensor_.local_y_Accel,
                  chassiSensor_.yaw,
                  chassiSensor_.timeStamp_10us);
}

/**
 * @brief 订阅全局速度控制消息回调
 * @param msg 来自 /acl_velocity 的底盘控制消息
 */
void usartConfig::GlobalVelSubCallback(const robot_communication::chassisControlConstPtr &msg)
{
    controlMotion_ = *msg;
}

/**
 * @brief 订阅 /cmd_vel_auto 的回调函数
 * 将 geometry_msgs::Twist 转换为内部控制结构体
 * @param msg Twist消息
 */
void usartConfig::CmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
    // 线速度
    controlMotion_.xSpeed = msg->linear.x;
    controlMotion_.ySpeed = msg->linear.y;

    // 当前版本中底盘角度不从cmd_vel传入，这里固定为0
    controlMotion_.chassisAngle = 0.0;

    // 将角速度 z 赋值给底盘角速度
    controlMotion_.chassisGyro = msg->angular.z;
}

/**
 * @brief 统计串口接收频率
 * 当前函数保留，用于调试接收频率和平均成功率
 */
void usartConfig::displayUsartFreq(void)
{
    freqFlag_ = 1;
    static uint32_t i = 0, time = 0, num = 0;
    static uint64_t flagSucc = 0;

    i++;
    if (i >= 1000)
    {
        if (freq_ >= 1000)
        {
            freq_ = 0;
        }

        i = 0;
        if (freq_ > 0)
        {
            flagSucc += freq_;
            num++;
            std::cout << static_cast<float>(flagSucc / num) << std::endl;
        }

        ROS_WARN("%d", freq_);
        freq_ = 0;
        time++;
        std::cout << time << std::endl;
    }
}