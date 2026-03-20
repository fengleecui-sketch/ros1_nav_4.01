/*
 * @Author: your name
 * @Date: 2023-04-25 09:17:45
 * @LastEditTime: 2023-04-25 20:03:14
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /Cost_Map/src/cost_map.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "cost_map.h"

cost_map::cost_map(/* args */)
{

}

cost_map::~cost_map()
{
	// 释放每行的内存空间
	for (int i = 0; i < rows; ++i) {
			delete[] deal_img_data[i];
	}
	// 释放二维指针的内存空间
	delete[] deal_img_data;

	// 释放内存
	for (int i = 0; i < rows; ++i) {
		delete[] img_data[i];
	}
	delete[] img_data;
}

void cost_map::InitialMap(cv::String imgpath,double _resolution,double _robot_radius)    //系统初始化
{
	path = imgpath;
	// img参数初始化 读取图片
	img = imread(path, IMREAD_GRAYSCALE);
	if (img.empty()) {
		std::cerr << "Failed to read image" << std::endl;
		while(1);		//没有图片程序不执行
	}

	rows = img.rows;	//获取行
	cols = img.cols;	//获取列

	cout<<"image rows: "<<rows<<endl;
	cout<<"image cols: "<<cols<<endl;

	// 初始化
	deal_img_data = new imag_array_define*[rows];
	for (int i = 0; i < rows; ++i) {
    // 分配每行的内存空间
    deal_img_data[i] = new imag_array_define[cols];
    for (int j = 0; j < cols; ++j) {
			// 对每个元素进行初始化
			deal_img_data[i][j].status = 0;
			deal_img_data[i][j].num = 0;
			deal_img_data[i][j].radius = (int)(robot_radius/resolution);
			deal_img_data[i][j].infla_flag = false;
    }
	}

	// 初始化图片获取的二维数组
	img_data = new uchar*[rows];
	for (int i = 0; i < rows; ++i) {
		img_data[i] = new uchar[cols];
	}
}

void cost_map::MapSave(void)
{
	// 将像素存储到二维数组中
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			img_data[i][j] = img.at<uchar>(i, j);
			if(img_data[i][j] == 255)
			{
				deal_img_data[i][j].num = 255;
				deal_img_data[i][j].status = 0;	// 不是障碍物还没有处理
				cout<<0;
			}
			else
			{
				deal_img_data[i][j].num = 0;
				deal_img_data[i][j].status = 1;	// 是障碍物还没处理
				// 没有膨胀
				deal_img_data[i][j].infla_flag = false;
				cout<<1;
			}
		}
		cout<<endl;
	}
	cout<<endl;
}

void cost_map::Map_Inflation(void)
{
	// 计时开始
	auto start_time = chrono::high_resolution_clock::now();

	// 画框，如果导入图片没有外边框就画一个
	for (int j = 0; j < cols; ++j) {
		deal_img_data[0][j].num = 0;
		deal_img_data[0][j].status = 1;	// 不是障碍物还没处理
	}
	for (int j = 0; j < cols; ++j) {
		deal_img_data[rows-1][j].num = 0;
		deal_img_data[rows-1][j].status = 1;	// 不是障碍物还没处理
	}
	for (int i = 0; i < rows; ++i) {
		deal_img_data[i][0].num = 0;
		deal_img_data[i][0].status = 1;	// 不是障碍物还没处理
	}
	for (int i = 0; i < rows; ++i) {
		deal_img_data[i][cols-1].num = 0;
		deal_img_data[i][cols-1].status = 1;	// 不是障碍物还没处理
	}

	// 对二维数组进行处理
	// 开始膨胀
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			// 只考虑障碍物附近开始画圆
			if(deal_img_data[i][j].status == 1 && deal_img_data[i][j].num == 0)
			{
				// deal_img_data[i][j].status == 1;	//代表已经处理过
				deal_img_data[i][j].radius = 10;	//半径取10
				//以 i j为圆心画实心圆
				for(int k=1;k<=deal_img_data[i][j].radius;k++)
				{
					for(int circle=0;circle < 63;circle++)
					{
						double x = k*sin(circle/10.0f);
						double y = k*cos(circle/10.0f);
						double axis_x = i+(int)x;
						double axis_y = j+(int)y;
						// 超出边界范围的，和之前不是障碍物本身但是被膨胀过的，跳过，提高代码运行效率防止出现bug
						if((int)axis_x > rows-1 || (int)axis_x < 0 || (int)axis_y > cols-1 || (int)axis_y<0 || 
							(deal_img_data[(int)axis_x][(int)axis_y].infla_flag == true &&
						   deal_img_data[(int)axis_x][(int)axis_y].status == 0 ))
						{
							continue;
						}
						else
						{
							deal_img_data[(int)axis_x][(int)axis_y].num = 0;
							deal_img_data[(int)axis_x][(int)axis_y].infla_flag = true;
							deal_img_data[i][j].infla_flag = true;	//该圆心已经膨胀完毕
						}
					}
				}
			}
		}
	}

	auto end_time = chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
	// 输出膨胀之后的数组
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			if(deal_img_data[i][j].num == 255)
			{
				cout<<0;
			}
			else if(deal_img_data[i][j].num == 0)
			{
				cout<<1;
			}
		}
		cout<<endl;
	}
	cout<<endl;

	cout<<"运行时间："<<duration.count()/1000<<"ms"<<endl;
}

void cost_map::OutputPng(void)
{
	// 将处理后的像素值存储回 Mat 对象
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			img.at<uchar>(i, j) = deal_img_data[i][j].num;
		}
	}

	// 保存图片
	imwrite("processed_image.png", img);
}