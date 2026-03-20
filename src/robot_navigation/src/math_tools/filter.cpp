/*
 * @Author: your name
 * @Date: 2023-09-07 09:27:28
 * @LastEditTime: 2023-09-07 11:25:20
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/src/math_tools/filter.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "math_tools/filter.h"

namespace math_tools
{
filter::filter(/* args */)
{
}

filter::~filter()
{
}

/**
  * @brief  二阶低通滤波函数       
  * @param[in]   对应传感器   
  * @param[in]   样本数据频率
  * @param[in]   截止频率
	* @date		2021/12/23
	* @author		TDT
  * @retval         none
	* @note		截止频率计算公式 20lgA(w) = -3db
											A(w) = 0.707 或者说根号二分之1
  */
void filter::LPF2pSetCutoffFreq(int index, float sample_freq, float cutoff_freq)
{
  LPF *lpf;
  lpf = &lpf4[index];
  float fr = 0;
  float ohm = 0;
  float c = 0;

  fr = sample_freq / cutoff_freq;
  ohm = tan(M_PI / fr);
  c = 1.0f + 2.0f * cos(M_PI / 4.0f) * ohm + ohm * ohm;

  /* 直接给出截止频率减少运算步骤 */
  lpf->_cutoff_freq1 = cutoff_freq;
  if (lpf->_cutoff_freq1 > 0.0f)
  {
    lpf->_b01 = ohm * ohm / c;
    lpf->_b11 = 2.0f * lpf->_b01;
    lpf->_b21 = lpf->_b01;
    lpf->_a11 = 2.0f * (ohm * ohm - 1.0f) / c;
    lpf->_a21 = (1.0f - 2.0f * cos(M_PI / 4.0f) * ohm + ohm * ohm) / c;
  }
}

/**
* @brief  二阶低通滤波函数       
* @param[in]   对应传感器   
* @param[in]   样本数据
* @retval 二阶低通滤波函数
*/
float filter::LPF2pApply(int index,float sample)
{
  LPF *lpf;
  lpf = &lpf4[index];
  float delay_element_0 = 0, output = 0;
  if (lpf->_cutoff_freq1 <= 0.0f)
  {
    // no filtering
    return sample;
  }
  else
  {
    delay_element_0 = sample - lpf->_delay_element_11 * lpf->_a11 - lpf->_delay_element_21 * lpf->_a21;
    // do the filtering
    if (isnan(delay_element_0) || isinf(delay_element_0))
    {
        // don't allow bad values to propogate via the filter
        delay_element_0 = sample;
    }
    output = delay_element_0 * lpf->_b01 + lpf->_delay_element_11 * lpf->_b11 + lpf->_delay_element_21 * lpf->_b21;

    lpf->_delay_element_21 = lpf->_delay_element_11;
    lpf->_delay_element_11 = delay_element_0;

    // return the value.  Should be no need to check limits
    return output;
  }  
}

/* data */
// 平滑滤波初始化
// 平滑阶数目
void filter::Smooth_Filter_Init(int K)
{
  filter_k = K;
  FILT_BUF = new double[filter_k];
  // 平滑滤波数组参数定义
  FILT_BUF[filter_k] = {0.0};
}

void filter::Smooth_Filter(double input,double &output)
{
  // 定义
  double sum_value = 0.0;

  for(int i=filter_k-1;i>=1;i--)
  {
    FILT_BUF[i] = FILT_BUF[i-1];
  }
  FILT_BUF[0] = input;
  for(int i=0;i<filter_k;i++)
  {
    sum_value += FILT_BUF[i];
  }

  output = sum_value/(double)filter_k;

}

}
