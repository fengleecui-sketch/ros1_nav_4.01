# 该程序目前实现的是.png地图图片根据机器人最大半径进行膨胀的功能
## 要求环境：opencv cmake make都要配置好
### 
1. opencv可以按照GPT的指示进行配置非常方便
2. cmake和make配置有些麻烦，可以根据以下链接：
    在哔哩哔哩搜索[xiaobing1016](https://www.bilibili.com/video/BV1fy4y1b7TC/?spm_id_from=333.788.top_right_bar_window_custom_collection.content.click&vd_source=45d17a7fb904c6c80f292c82cbb3107d)，视频讲的非常细致认真
## 使用方法，在配置好以上环境之后搭建工程make编译通过之后在build空间下输入：./cost_map /home/lilei/桌面/Cost_Map/temp.png
上面的路径根据自己的情况进行修改，这里就不多说了
```c++
lilei@lilei:~/桌面/Cost_Map/build$ make
Scanning dependencies of target cost_map
[ 33%] Building CXX object CMakeFiles/cost_map.dir/src/cost_map.cpp.o
[ 66%] Linking CXX executable cost_map
[100%] Built target cost_map
```

## 膨胀部分代码：逻辑很简单，就是根据每个障碍物的点去按照膨胀半径画实心圆
```c++
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
```
## 扩展：以后代价地图的处理包括维诺图等地图处理可在此代码基础上进行改进
