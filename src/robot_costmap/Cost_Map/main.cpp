/*
 * @Author: your name
//  * @Date: 2023-04-25 09:17:45
 * @LastEditTime: 2023-04-25 19:56:32
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /Cost_Map/main.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "cost_map.h"
#include "chrono"

// imag_array_define **deal_img_data;
// uchar** img_data;

int main(int argc, char** argv)
{
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " image_path" << std::endl;
		return 1;
	}

	cost_map test;

	test.InitialMap(argv[1],0.04,0.4);
	test.MapSave();
	test.Map_Inflation();
	test.OutputPng();
	
	return 0;
}

