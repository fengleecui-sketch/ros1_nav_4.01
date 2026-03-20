/*
 * @Author: your name
 * @Date: 2023-04-25 09:17:45
 * @LastEditTime: 2023-04-25 19:56:45
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /Cost_Map/include/cost_map.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __COST_MAP_H
#define __COST_MAP_H

#include "iostream"
#include "opencv2/opencv.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

using namespace std;
using namespace cv;

// 定义图片处理相关数组
typedef struct imag_array
{
	uint16_t status;		//当前状态，0未处理，1是已经处理
	uint16_t num;		//赋值
	uint16_t radius;	//半径
	bool     infla_flag;	//膨胀标志位

  imag_array(){};
  ~imag_array(){};
}imag_array_define;

class cost_map
{
protected:
	/* data */
	cv::String path;  //图片路径
	uint16_t rows;    //行
	uint16_t cols;    //列
	Mat img;          //用于opencv提取图片的数组

	imag_array_define **deal_img_data;
	uchar **img_data;

	double resolution;	//输入分辨率，即一格代表多少m，这里取
	double robot_radius;	//机器人半径用于膨胀，单位m

public:
	cost_map(/* args */);
	~cost_map();
	// 图片路径，地图分辨率，机器人最大半径
	void InitialMap(cv::String imgpath,double _resolution,double _robot_radius);    //系统初始化
	// 用来膨胀地图  并打印输出
	void Map_Inflation(void); //
	// 输出png图片
	void OutputPng(void);     //输出png图片
	// 保存当前地图图片  并打印输出
	void MapSave(void);				//地图保存
};


#endif // __COST_MAP_H

