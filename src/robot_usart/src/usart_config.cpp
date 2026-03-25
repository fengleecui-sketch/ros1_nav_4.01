/*
 * @Author: your name
 * @Date: 2023-03-21 09:57:25
 * @LastEditTime: 2024-04-19 00:10:04
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_usart/src/usart_config.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "robot_usart/usart_config.h"
//log日志
#include "easylog/easylogging++.h"

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

INITIALIZE_EASYLOGGINGPP

#define usbUsart  "/dev/ttyUSB0"
// #define usbUsart  "/dev/ttyACM0"

usartConfig::usartConfig() : private_node("~")
{
  // 串口配置
  Usart_Config();
  
  ros::Rate r(50);
  while (ros::ok())
  {
    ros::spinOnce();
    // displayUsartFreq();
      // 发布ROS消息的代码
    r.sleep();
  }

  // 串口关闭
  UsartClose();
}

usartConfig::~usartConfig()
{

}


void usartConfig::controlCmdSendCallback(const ros::TimerEvent &)
{
  try
  {
    //--[0,1]帧头
    usartTxBuffer[0] = 0x03;
    usartTxBuffer[1] = 0xfc;
    //--[2,3]车体x线速度
    usartTxBuffer[2] = (int)((controlMotion.xSpeed + 4.000f) * 1000) >> 8;
    usartTxBuffer[3] = (int)((controlMotion.xSpeed + 4.000f) * 1000) & 0xff;
    //--[4,5]车体y线速度
    usartTxBuffer[4] = (int)((controlMotion.ySpeed + 4.000f) * 1000) >> 8;
    usartTxBuffer[5] = (int)((controlMotion.ySpeed + 4.000f) * 1000) & 0xff;
    //--[6,7]底盘 yaw值
    usartTxBuffer[6] = (int)((controlMotion.chassisAngle*RAD_TO_ANGLE + 180.0f) * 100) >> 8;
    usartTxBuffer[7] = (int)((controlMotion.chassisAngle*RAD_TO_ANGLE + 180.0f) * 100) & 0xff;
    //--[8,9,10]底盘 角速度值
    usartTxBuffer[8] = (int)((controlMotion.chassisGyro*RAD_TO_ANGLE + 360.0f) * 100) >> 16;
    usartTxBuffer[9] = (int)((controlMotion.chassisGyro*RAD_TO_ANGLE + 360.0f) * 100) >> 8;
    usartTxBuffer[10] = (int)((controlMotion.chassisGyro*RAD_TO_ANGLE + 360.0f) * 100) & 0xff;

    // 发送运行模式 1是发送的全向运动速度 0是发送的直接速度控制
    usartTxBuffer[11] = use_global;

    usart_check.Append_CRC16_Check_Sum(usartTxBuffer,TX_LENGTH);
    // CRC校验
    if(serial_port_->is_open())
    {
      boost::asio::write(*serial_port_, boost::asio::buffer(usartTxBuffer, TX_LENGTH));
    }
  }
  catch (...)
  {
    cout << "串口无法链接" << endl;
    exit(0);
  }
}

void usartConfig::displayUsartFreq(void)
{
  freqFlag = 1;
  // 1s计数
  static uint32_t i = 0,time = 0,num = 0;
  static uint64_t flagSucc = 0;
  i++;
  if(i >= 1000)
  {
    // 超过1000不相信
    if(freq >= 1000)
    {
      freq = 0;
    }
    
    i = 0;
    if(freq > 0)
    {
      flagSucc += freq;
      num ++;
      cout<<(float)(flagSucc/num)<<endl;
    }
    ROS_WARN("%d\r\n",freq);
    freq = 0;
    time ++;
    cout<<time<<endl;
  }
}

// 读取串口数据
int usartConfig::ReadUsart() {
  // 当串口打开的状况下
  if (serial_port_->is_open()) {
    while (true) {
      try {
        // 当开始接收
        if (recv_buff.ON_RECV_HEADER) {
          // 接收字符串长度大于1
          if (recv_buff.recv_len < 1) {
            // 开始读取数据长度
            int recv_len = boost::asio::read(
                *serial_port_, boost::asio::buffer(recv_buff.buff, 1));
            // 当长度小于等于0的时候
            if (recv_len <= 0) {
              return -3;  // Read No Data
            }
            recv_buff.recv_len = recv_len;
          }
          // 当第一位为frame_header 且没有犯错误的情况下，
          if (recv_buff.buff[0] == frame_header && (!recv_buff.WRONG_TICK)) {
            recv_buff.ON_RECV_HEADER = false;
          } else {
            // 没有犯错
            recv_buff.WRONG_TICK = false;
            bool find_header = false;
            // 对接收到的数据进行判断
            for (int i = 1; i < recv_buff.recv_len; i++) {
              // 当接收的数据中位为frame_header的时候
              if (recv_buff.buff[i] == frame_header) {
                // 重新计算字符串长度
                recv_buff.recv_len = recv_buff.recv_len - i;
                // 将字符串进行复制
                memcpy(recv_buff.buff, recv_buff.buff + i, recv_buff.recv_len);
                // 
                recv_buff.ON_RECV_HEADER = false;
                // 接收类型正确
                recv_buff.ON_RECV_TYPE = true;
                // 接收数据正确
                recv_buff.ON_RECV_DATA = true;
                // 寻找到了header
                find_header = true;
                break;
              }
            }
            // 当没有header
            if (!find_header) {
              // 串口接收buff复位
              recv_buff.reset();
              // header设定为正确
              recv_buff.ON_RECV_HEADER = true;
              recv_buff.ON_RECV_TYPE = true;
              recv_buff.ON_RECV_DATA = true;
            }
            continue;
          }
        }

        // 当接收数据类型正确
        if (recv_buff.ON_RECV_TYPE) {
          // 当buff长度小于2
          if (recv_buff.recv_len < 2) {
            // 再次读取串口数据
            int recv_len = boost::asio::read(
                *serial_port_, boost::asio::buffer(recv_buff.buff + 1, 1));
            // 如果接收数据长度还小于0 返回值-3
            if (recv_len <= 0) {
              return -3;
            }
            recv_buff.recv_len = 1 + recv_len;
          }
          // 第二位的标志在 0x01 到 0x05之间
          if (recv_buff.buff[1] == 0xfa) {
            recv_buff.ON_RECV_TYPE = false;
          } else {
            // 犯错有问题
            recv_buff.set_wrong_tick();
            continue;
          }
        }

        // 
        if (recv_buff.ON_RECV_DATA) {
          // 获取数据长度
          int expected_recv_len = RX_LENGTH;
          // 如果接收到的数据长度小于期望的数据长度
          if (recv_buff.recv_len < expected_recv_len) {
            // 读取串口的接收长度
            int recv_len = boost::asio::read(
                *serial_port_,
                boost::asio::buffer(recv_buff.buff + recv_buff.recv_len,
                                    expected_recv_len - recv_buff.recv_len));
            if (recv_len <= 0) {
              return -3;
            }
            // 接收到的数据长度 为 原长度+读取到的字符串长度
            recv_buff.recv_len = recv_buff.recv_len + recv_len;
          }
          // 当接收到的长度小于期望长度返private_node回错误码-2 
          if (recv_buff.recv_len < expected_recv_len) {
            return -2;  // Read Length Error
          }
          // 进行CRC校验,正确则返回true 
          if (usart_check.Verify_CRC16_Check_Sum(recv_buff.buff, expected_recv_len)) {
              // 对接收数据进行解码
              recData_Decode();
              /* 时间戳解算 */
              uint32_t time_stamp_10us = recv_buff.buff[12] << 28 | recv_buff.buff[13] << 24 | recv_buff.buff[14] << 16
                                        | recv_buff.buff[15] << 8 | recv_buff.buff[16];

              // cout<<time_stamp_10us<<endl;
              if(freqFlag == 1)   //当标志位置1此时开始打印
              {
                freq++;
              }
            recv_buff.ON_RECV_DATA = false;
          } else {  //CRC校验错误
            recv_buff.set_wrong_tick();
            return -1;  // CRC Error
          }
        }
        return recv_buff.buff[1];
      } catch (...) {
        return -3;
      }
    }
  } else {
    return -4;  // Usart Offline
  }
}

void usartConfig::UsartClose() {
  serial_port_->close();
  on_running = false;
}

// 串口重启
void usartConfig::UsartRestart() 
{ Usart_Config(); }

// 串口接收线程
void usartConfig::RecvThread() {
  // double last_recv_time = tdttoolkit::Time::GetTimeNow();
  while (on_running) {
    thread_locker.lock();
    int ret = ReadUsart();
    thread_locker.unlock();
    if (ret == -3) {
      // double cnt_time = tdttoolkit::Time::GetTimeNow();
      // if (fabs(cnt_time - last_recv_time) > 1e6) {
        // last_recv_time = cnt_time;
        // TDT_WARNING("串口获取超时,重启串口");
        UsartRestart();
      // }
      usleep(100);  // 两次主动获取串口信息之间延时，单位us
    } else {
      // last_recv_time = tdttoolkit::Time::GetTimeNow();
    }
    if (ret == -1) {
      // TDT_WARNING("CRC校验失败");
      // usleep(10);
    }
    if (ret == -4) {
      // TDT_WARNING("串口离线,重启串口");
      UsartRestart();
      usleep(100);
    }
    if (ret > 0) {
      // shared_data::DataRecver[ret]->ParseData(recv_buff.buff);
      recv_buff.reset();
    }
  }
}


// 串口配置
void usartConfig::Usart_Config(void)
{
  //--json参数读取配置
  ifstream(ros::package::getPath("robot_usart") + "/src/Jason/param.json") >> param;
  //--shell终端赋权限
  string linuxCmd;
  linuxCmd = linuxCmd + "echo " + (string)param["uartParam"].at("passWord") + " | sudo -S chmod 777 " + (string)param["uartParam"].at("usbUsart");
  int res = system(linuxCmd.c_str());
  if(res == -1)
  {
    ROS_WARN("linuxCmd Error!");
  }
  //--easylog日志记录配置
  el::Configurations conf(ros::package::getPath("robot_usart") + "/src/easylog/log.conf");
  el::Loggers::reconfigureAllLoggers(conf);

  boost::asio::io_service iosev;
   serial_port_ = new serial_port(iosev, "/dev/ttyUSB0");
  // serial_port_ = new serial_port(iosev, "/dev/ttyACM0");

    // 配置串口参数
  // 设定串口波特率
  serial_port_->set_option(serial_port::baud_rate(460800));
  // 设置不限流控制，可以以最大速率进行传输
  serial_port_->set_option(
      serial_port::flow_control(serial_port::flow_control::none));
  // 无奇偶校验
  serial_port_->set_option(serial_port::parity(serial_port::parity::none));
  // 一个停止位
  serial_port_->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  // 数据位设置为8位
  serial_port_->set_option(serial_port::character_size(8));

  // 加载参数
  private_node.param("usart_node/use_global",use_global,false); // 是否仿真

  cout<<"use_global:"<<use_global<<endl;

  // 订阅运动规划部分发布的速度消息
  //globalVelSub = private_node.subscribe("/acl_velocity",10,&usartConfig::GlobalVelSubCallback,this);
  globalVelSub = private_node.subscribe("/acl_velocity",10,&usartConfig::GlobalVelSubCallback,this);

  //cmdVelSub = private_node.subscribe("/cmd_vel", 10, &usartConfig::CmdVelCallback, this);

  cmdVelSub = private_node.subscribe("/cmd_vel_auto", 10, &usartConfig::CmdVelCallback, this);

  // 50hz定时器
  cmdTimer = private_node.createTimer(ros::Duration(0.02),&usartConfig::controlCmdSendCallback,this);
  // 以200hz定时器发布传感器消息
  sensorTimer = private_node.createTimer(ros::Duration(0.005),&usartConfig::PubSensor_DataSendCallback,this);

  // 发布底盘传感器消息
  SensorDataPub = private_node.advertise<robot_communication::sensorData>("/chassis_sensor_data",1);



  //
  on_running = true;
  // RecvThread();
  auto recv_thread = std::thread(&usartConfig::RecvThread,this);
  recv_thread.detach();
}

void usartConfig::recData_Init(void)
{
  recSensor.chassix_x_linear_velocity = 0.0f;
  recSensor.chassis_y_linear_velocity = 0.0f;

  recSensor.chassis_x_accelerate = 0.0f;
  recSensor.chassis_y_accelerate = 0.0f;

  recSensor.chassis_yaw = 0.0f;

  recSensor.time_stamp_10us = 0;
}

void usartConfig::recData_Decode(void)
{
  /* 线速度解算 */
  recSensor.chassix_x_linear_velocity = ((recv_buff.buff[2] << 8 | recv_buff.buff[3]) - 4000.0f)/1000.0f;
  recSensor.chassis_y_linear_velocity = ((recv_buff.buff[4] << 8 | recv_buff.buff[5]) - 4000.0f)/1000.0f;
  /* 加速度解算 */
  recSensor.chassis_x_accelerate = ((recv_buff.buff[6] << 8 | recv_buff.buff[7]) - 5000.0f)/1000.0f;
  recSensor.chassis_y_accelerate = ((recv_buff.buff[8] << 8 | recv_buff.buff[9]) - 5000.0f)/1000.0f;
  /* 角速度解算 */
  recSensor.chassis_yaw = ((recv_buff.buff[10] << 8 | recv_buff.buff[11]) - 18000.0f)/100.0f;
  /* 时间戳解算 */
  recSensor.time_stamp_10us = recv_buff.buff[12] << 28 | recv_buff.buff[13] << 24 | recv_buff.buff[14] << 16
                              | recv_buff.buff[15] << 8 | recv_buff.buff[16];

  // std::cout << fontColorWhite << "v_x" <<recSensor.chassix_x_linear_velocity<<" "
  //                             << "v_y" <<recSensor.chassis_y_linear_velocity<<" "
  //                             << "a_x" <<recSensor.chassis_x_accelerate<<" "
  //                             << "a_y" <<recSensor.chassis_y_accelerate<<" "
  //                             << "yaw" <<recSensor.chassis_yaw <<""
  // <<"[" <<"time_stamp:"<<recSensor.time_stamp_10us << "]"
  // << std::endl;
}

// 发布传感器消息
void usartConfig::PubSensor_DataSendCallback(const ros::TimerEvent &)
{
  // 坐标系不太一样修改一下
  chassiSensor.local_x_Veloc = recSensor.chassix_x_linear_velocity;
  chassiSensor.local_y_Veloc = recSensor.chassis_y_linear_velocity;

  chassiSensor.local_x_Accel = recSensor.chassis_x_accelerate;
  chassiSensor.local_y_Accel = recSensor.chassis_y_accelerate;

  // 转换成弧度制
  chassiSensor.yaw = recSensor.chassis_yaw/57.3f;

  chassiSensor.timeStamp_10us = recSensor.time_stamp_10us;
  SensorDataPub.publish(chassiSensor);
}

// 全局速度消息订阅
void usartConfig::GlobalVelSubCallback(const robot_communication::chassisControlConstPtr &msg)
{
  controlMotion = *msg;

  //cout<<controlMotion<<endl;
}

// 串口发送函数 controlCmdSendCallback()
void usartConfig::CmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
  // 按你的串口发送格式填充 controlMotion
  controlMotion.xSpeed = msg->linear.x;
  controlMotion.ySpeed = msg->linear.y;

  // 角度/角速度（如果你底盘不需要角度，就先置 0）
  controlMotion.chassisAngle = 0.0;
  controlMotion.chassisGyro  = msg->angular.z;

  cout << controlMotion << endl;
}




