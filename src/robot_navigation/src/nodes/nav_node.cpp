/***
 * @                       _oo0oo_
 * @                      o8888888o
 * @                      88" . "88
 * @                      (| -_- |)
 * @                      0\  =  /0
 * @                    ___/`---'\___
 * @                  .' \\|     |// '.
 * @                 / \\|||  :  |||// \
 * @                / _||||| -:- |||||- \
 * @               |   | \\\  - /// |   |
 * @               | \_|  ''\---/''  |_/ |
 * @               \  .-\__  '-'  ___/-. /
 * @             ___'. .'  /--.--\  `. .'___
 * @          ."" '<  `.___\_<|>_/___.' >' "".
 * @         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 * @         \  \ `_.   \_ __\ /__ _/   .-` /  /
 * @     =====`-.____`.___ \_____/___.-`___.-'=====
 * @                       `=---='
 * @
 * @
 * @     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * @
 * @           佛祖保佑     永不宕机     永无BUG
 * @
 * @Author: your name
 * @Date: 2022-12-04 12:35:47
 * @LastEditTime: 2022-12-29 16:23:57
 * @LastEditors: your name
 * @Description:
 * @FilePath: /tianbot_mini/src/astar_super/src/demo_node.cpp
 * @可以输入预定的版权声明、个性签名、空行等
 */
#include "path_searcher/nav_app.h"

/// @brief
/// @param argc
/// @param argv
/// @return
int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "nav_node");

  navSolution nav(5);
  return 0;
}
