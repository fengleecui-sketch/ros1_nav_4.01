/*
 * @Author: your name
 * @Date: 2023-09-07 09:29:48
 * @LastEditTime: 2023-09-07 11:23:41
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/include/math_tools/filter.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __FILTER_H
#define __FILTER_H

#include "iostream"
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

namespace math_tools
{

struct _LPF
{
    float    _cutoff_freq1;
    float           _a11;
    float           _a21;
    float           _b01;
    float           _b11;
    float           _b21;
    float           _delay_element_11;        // buffered sample -1
    float           _delay_element_21;        // buffered sample -2
	
};
typedef struct _LPF  LPF;

class filter
{
private:
  int filter_k;    //滤波阶数
  double * FILT_BUF;    //用于存储滤波数值的数组
  LPF lpf4[12];
public:
  /* data */
  // 平滑滤波初始化
  // 平滑阶数目
  void Smooth_Filter_Init(int K);
  // 平滑滤波
  void Smooth_Filter(double input,double &output);

  // 2阶低通滤波设定参数
/**
  * @brief  二阶低通滤波函数       
  * @param[in]   数据量    
  * @param[in]   样本数据频率
  * @param[in]   需要滤掉的高频频率 
  * @retval         none
  */
  void LPF2pSetCutoffFreq(int index, float sample_freq, float cutoff_freq);

  /**
  * @brief  二阶低通滤波函数       
  * @param[in]   对应传感器   
  * @param[in]   样本数据
  * @retval 二阶低通滤波函数
  */
  float LPF2pApply(int index,float sample);


  filter(/* args */);
  ~filter();
};


}

#endif  //

