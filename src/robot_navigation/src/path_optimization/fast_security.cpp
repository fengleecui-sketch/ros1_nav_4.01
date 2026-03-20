#include "path_optimization/fast_security.h"

Fast_Security::Fast_Security(/* args */)
{

}

Fast_Security::~Fast_Security()
{
}

void Fast_Security::InitParams(ros::NodeHandle &nh)
{
  // 查看访问点
  visitNodesPub = nh.advertise<visualization_msgs::Marker>("/Visited_nodes", 10);
  // 查看选出来的优化点
  visitOptpathPub = nh.advertise<visualization_msgs::Marker>("/Opt_path_visit", 10);
  // 查看采样点
  sampleNodesPub = nh.advertise<visualization_msgs::Marker>("/Sample_node",10);
  // 查看连续点
  continuNodesPub = nh.advertise<visualization_msgs::Marker>("/Continu_node",10);
  // 查看补充扩展节点
  suppleMentPub = nh.advertise<visualization_msgs::Marker>("/Supplement_node",10);

  // 获取安全走廊搜索长度或者说宽度
  // 最小按照实车半径去给,这里保守一点给2.0
  nh.param("Fast_security/search_length",search_length,2.0);
  // 获取连续部分走廊系数
  // 距离场系数
  nh.param("Fast_security/Kcontinu_esdf",Kcontinu_esdf,1.0);
  // 到上一个点距离代价系数
  nh.param("Fast_security/Kcontinu_ldis",Kcontinu_ldis,1.0);
  // 到终点部分距离代价系数
  nh.param("Fast_security/Kcontinu_edis",Kcontinu_edis,1.0);

  // 获取间断部分走廊系数
  // 距离场系数
  nh.param("Fast_security/Kinterru_esdf",Kinterru_esdf,1.0);
  // 到上一个点距离代价系数
  nh.param("Fast_security/Kinterru_ldis",Kinterru_ldis,1.0);
  // 到终点部分距离代价系数
  nh.param("Fast_security/Kinterru_edis",Kinterru_edis,1.0);
  // 重合点系数
  nh.param("Fast_security/Kinterru_rep",Kinterru_rep,1.0);

  // 获取阶梯部分走廊系数
  // 距离场系数
  nh.param("Fast_security/Kstep_esdf",Kstep_esdf,1.0);
  // 到上一个点距离代价系数
  nh.param("Fast_security/Kstep_ldis",Kstep_ldis,1.0);
  // 到终点部分距离代价系数
  nh.param("Fast_security/Kstep_edis",Kstep_edis,1.0);
  // 重合点系数
  nh.param("Fast_security/Kstep_rep",Kstep_rep,1.0);

}

// 复位访问的节点
void Fast_Security::resetVisited(void)
{
  // 清空优化路径
  optpath.clear();
  // 连续点集合
  continuAssem.clear();
  // 垂直与连续点的向量清空
  vercontAssem.clear();

  // 分段清0
  dividePath.clear();
  // 垂直于分段的向量清0
  verdividePath.clear();
  // 连续分段
  continuDivid.clear();
  // 垂直于连续分段
  vercontinuDivid.clear();

  // 栅格路径清0
  origingridPath.clear();
  // 优化路径清0
  optedpath.clear();

  visualOptpoints.clear();
}

/**
 * @brief 栅格地图坐标系转世界坐标系
 * @param mx   地图坐标x
 * @param my   地图坐标y
 * @param wx   世界坐标x
 * @param wy   世界坐标y
 * @return
 * @attention
 * @todo
 * */
Vector2d Fast_Security::mapToWorld(Vector2i mapt) const
{
  double wx,wy;
  wx = origin_x + (mapt[0] + 0.5) * resolution;
  wy = origin_y + (mapt[1] + 0.5) * resolution;

  return Vector2d(wx,wy);
}

/**
 * @brief 世界坐标系转栅格地图坐标系
 * @param wx   世界坐标x
 * @param wy   世界坐标y        if(isOccupied(pointS))
      {
        break;
      }
  * @param mx   地图坐标x
  * @param my   地图坐标y
  * @return
  * @attention
  * @todo
  * */
Vector2i Fast_Security::worldToMap(Vector2d worldpt) const
{
  int mx,my;

  mx = (int)(1.0 * (worldpt[0] - origin_x) * resolution_inv);
  my = (int)(1.0 * (worldpt[1] - origin_y) * resolution_inv);

  return Vector2i(mx,my);
}

// 判断是否在地图中
bool Fast_Security::isInMap(Eigen::Vector2d worldpt)
{
  if (worldpt(0) < origin_x || worldpt(1) < origin_y)
  {
    // cout << "less than min range!" << endl;
    // cout << "world point: "<<worldpt(0)<<"  "<<worldpt(1)<<endl;
    return false;
  }

  if (worldpt(0) > mapover_x || worldpt(1) > mapover_y)
  {
    // cout << "larger than max range!" << endl;
    // cout << "world point: "<<worldpt(0)<<"  "<<worldpt(1)<<endl;
    return false;
  }

  return true;
}

// 判断点是否被占据的具体实现形式
bool Fast_Security::isOccupied(const Vector2d worldpt) const
{
  // 转换为栅格坐标
  Vector2i gridmap = worldToMap(worldpt);
  // 判断是否被占据 或者说是不是障碍物
  int idx = gridmap[0];
  int idy = gridmap[1];

  return (idx < grid_map_x && idy < grid_map_y && (mapdata[idx + idy * grid_map_x] == OCCUPIED));
}

// 判断点是否被占据的具体实现形式
bool Fast_Security::isOccupied(const Vector2i gridpt) const
{
  // 判断是否被占据 或者说是不是障碍物
  int idx = gridpt[0];
  int idy = gridpt[1];

  return (idx < grid_map_x && idy < grid_map_y && (mapdata[idx + idy * grid_map_x] == OCCUPIED));
}

// 设定访问节点
void Fast_Security::setVisited(const Vector2i gridpt)
{
  // 判断是否被占据 或者说是不是障碍物
  int idx = gridpt[0];
  int idy = gridpt[1];    

  search_data[idx + idy * grid_map_x] += 1;
}

// 判断点是否重复
bool Fast_Security::isRepeated(const Vector2i gridpt) const
{
  // 判断是否被占据 或者说是不是障碍物
  int idx = gridpt[0];
  int idy = gridpt[1];

  if (search_data[idx + idy * grid_map_x] >= 2)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// 获取访问的次数
int Fast_Security::getVisitedNum(const Vector2i gridpt) const
{
  // 判断是否被占据 或者说是不是障碍物
  int idx = gridpt[0];
  int idy = gridpt[1];

  return search_data[idx + idy * grid_map_x];
}

// 获取当前点的势场数值
int8_t Fast_Security::getESDFvalue(Vector2i gridpt)
{
  // 判断是否被占据 或者说是不是障碍物
  int idx = gridpt[0];
  int idy = gridpt[1];

  return (mapdata[idx + idy * grid_map_x]);
}

// 判断向量是否共线
bool Fast_Security::isCollinear(Vector2i vector1,Vector2i vector2,bool strict)
{
  if(strict == true)
  {
    // 判断是否共线 a=(p1,p2) b=(q1,q2) 共线条件 p1*q2=p2*q1
    if(vector1(0)*vector2(1) == vector1(1)*vector2(0))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    double angle = calVectorAngle(vector1,vector2);
    // 不严格共线要求 这里把角度限制在15度
    if(angle >= 0 && angle < PI/16)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

// 判断向量是否共线
bool Fast_Security::isCollinear(Vector2d vector1,Vector2d vector2,bool strict)
{
  if(strict == true)
  {
    // 判断是否共线 a=(p1,p2) b=(q1,q2) 共线条件 p1*q2=p2*q1
    if(vector1(0)*vector2(1) == vector1(1)*vector2(0))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    double angle = calVectorAngle(vector1,vector2);
    // 不严格共线要求 这里把角度限制在15度
    if(angle >= 0 && angle < PI/16)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

// 计算单位向量
Vector2d Fast_Security::calUnitvector(Vector2i unitv)
{
  double x = double(unitv[0]);
  double y = double(unitv[1]);
  Vector2d unitVector = Vector2d(x,y);
  // 计算单位向量
  unitVector = unitVector * 1.0f/(sqrt(pow(abs(x),2)+pow(abs(y),2)));
  return unitVector;
}

// 计算单位向量
Vector2d Fast_Security::calUnitvector(Vector2d unitv)
{
  // 计算单位向量
  unitv = unitv * 1.0f/(sqrt(pow(abs(unitv[0]),2)+pow(abs(unitv[1]),2)));
  return unitv;
}

// 计算两个点之间的长度欧氏距离
double Fast_Security::calPointLength(Vector2i vector1,Vector2i vector2)
{
  return (sqrt((vector1[0]-vector2[0])*(vector1[0]-vector2[0])+(vector1[1]-vector2[1])*(vector1[1]-vector2[1])));
}

// 计算两个点之间的长度欧氏距离
double Fast_Security::calPointLength(Vector2d vector1,Vector2d vector2)
{
  return (sqrt((vector1[0]-vector2[0])*(vector1[0]-vector2[0])+(vector1[1]-vector2[1])*(vector1[1]-vector2[1])));
}


// 计算向量之间的夹角
double Fast_Security::calVectorAngle(Vector2i vector1,Vector2i vector2)
{
  // 先单位化
  Vector2d vectorFirst = calUnitvector(vector1);
  // 
  Vector2d vectorSecond = calUnitvector(vector2);

  // 向量乘积
  double vector_angle = vectorFirst[0]*vectorSecond[0] + vectorFirst[1]*vectorSecond[1];
  // 计算夹角
  return acos(vector_angle);
  // return vector_angle/3.1415926;
}

// 计算向量之间的夹角
double Fast_Security::calVectorAngle(Vector2d vector1,Vector2d vector2)
{
  // 先单位化
  Vector2d vectorFirst = calUnitvector(vector1);
  // 
  Vector2d vectorSecond = calUnitvector(vector2);

  // 向量乘积
  double vector_angle = vectorFirst[0]*vectorSecond[0] + vectorFirst[1]*vectorSecond[1];
  // 计算夹角
  return acos(vector_angle);
  // return vector_angle/3.1415926;
}

// 计算向量之间的乘积
double Fast_Security::calVectorProduct(Vector2i vector1,Vector2i vector2)
{
  // 先单位化
  Vector2d vectorFirst = calUnitvector(vector1);
  // 
  Vector2d vectorSecond = calUnitvector(vector2);

  // 向量乘积
  double vector_product = vectorFirst[0]*vectorSecond[0] + vectorFirst[1]*vectorSecond[1];

  return vector_product;
}

// 将世界坐标点转换成栅格坐标系下的整数点
vector<Vector2i> Fast_Security::worldPathToGridPath(vector<Vector2d> worldPath)
{
  int worldLength = worldPath.size();
  vector<Vector2i> GridPath;
  for(int i=0;i<worldLength;i++)
  {
    Vector2i gridNode = worldToMap(worldPath[i]);
    GridPath.push_back(gridNode);
  }
  
  // 返回栅格子坐标系
  return GridPath;
}

// 将栅格坐标系下的整数点转换成世界坐标系下的点
vector<Vector2d> Fast_Security::GridPathToWorldPath(vector<Vector2i> gridPath)
{
  int worldLength = gridPath.size();
  vector<Vector2d> worldPath;
  for(int i=0;i<worldLength;i++)
  {
    Vector2d worldNode = mapToWorld(gridPath[i]);
    worldPath.push_back(worldNode);
  }
  
  // 返回栅格子坐标系
  return worldPath;  
}

// 搜索障碍物
// searchLength 搜索长度
// point 扩展起点
// vertical_m 垂直平分线
// add_num 向上扩展截止
// min_num 向下扩展截止
void Fast_Security::Search_Obstacle(double searchLength,Vector2i point,Vector2i vertical_m,int &add_num,int &min_num)
{
  int add_temp = search_num/2;
  int min_temp = search_num/2;
  
  // 累加求解
  for(int j=1;j<searchLength/2+1;j++)
  {
    Vector2i pointS;    //搜索的节点
    // 求出过B点且与AC垂直平分线段上的点
    pointS = point + j*vertical_m;
    // 判断求解的点是否在障碍物上
    if(isOccupied(pointS))
    {
      // 
      add_temp = j;
      break;
    }
  }

  // 累减求解
  for (int j=1;j<searchLength/2+1;j++)
  {

    Vector2i pointS;    //搜索的节点
    // 求出过B点且与AC垂直平分线段上的点
    pointS = point - j*vertical_m;
    // 判断求解的点是否在障碍物上
    if(isOccupied(pointS))
    {
      min_temp = j;
      break;
    }
  }

  add_num = add_temp;
  min_num = min_temp;
}

// 节点扩展
// addsearch 累加搜索长度
// minsearch 累减搜索长度
// point 扩展起点
// vertical_m 垂直平分线
// pointExtension 返回的点集合
void Fast_Security::Node_Extension(int addsearch,int minsearch,Vector2i point,Vector2i vertical_m,vector<Vector2i> &pointExtension)
{
  // 判断求解的点是否在障碍物上
  // 设定为访问节点
  setVisited(point);
  // 获取当前点
  pointExtension.push_back(point);
  // num-> i
  // searchLength -> search_num
  // point -> pointB
  // pointExtension -> pointAssem
  // 求解出向量BM之后，开始进行搜索
  // 搜索原理过B点的向量BM上固定长度length上以分辨率resolution搜索出几个点
  // 对比每个点在地图中的势场数值大小
  // 累加求解
  for(int j=1;j<addsearch+1;j++)
  {
    Vector2i pointS;    //搜索的节点
    // 求出过B点且与AC垂直平分线段上的点
    pointS = point + j*vertical_m;
    // 判断求解的点是否在障碍物上
    if(isOccupied(pointS))
    {
      break;
    }
    else
    {
      // 设定为访问节点
      setVisited(pointS);
      // 获取当前点
      pointExtension.push_back(pointS);
    }
  }

  // 累减求解
  for (int j=1;j<minsearch+1;j++)
  {
    Vector2i pointS;    //搜索的节点
    // 求出过B点且与AC垂直平分线段上的点
    pointS = point - j*vertical_m;
    // 判断求解的点是否在障碍物上
    if(isOccupied(pointS))
    {
      break;
    }
    else
    {
      // 设定为访问节点
      setVisited(pointS);
      // 获取当前点
      pointExtension.push_back(pointS);
    }
  }
}

// 中断节点扩展
// searchLength 搜索长度
// point 扩展起点
// vertical_m 垂直平分线
// pointExtension 返回的点集合
void Fast_Security::Inflect_Extension(int test,vector<Vector2i> &front1Extension,vector<Vector2i> &front2Extension,
vector<Vector2i> &after1Extension,vector<Vector2i> &after2Extension)
{
  // 前一个点扩展
  front1Extension.clear();
  front2Extension.clear();
  // 后一个点扩展
  after1Extension.clear();
  after2Extension.clear();

  // 对于中段部分搜索步长的下限限制
  if(max_step_vec[test] < search_num/4)
  {
    max_step_vec[test] = search_num/4;
  }
  if(max_step_vec[test+1] < search_num/4)
  {
    max_step_vec[test+1] = search_num/4;
  }
  
  // 向前
  for(int step = 0;step<max_step_vec[test]+search_num/8;step++)
  {
    // 一系列的点
    Vector2i tempoint = frontVec[test] + step*frontToafterVec[test];
    
    if(!isOccupied(tempoint))
    {
      // 以当前点b为中心进行方块扩展
      Node_Extension(inflect_add_search[4*test],inflect_min_search[4*test],tempoint,verfrontToafterVec[test],front1Extension);
    }
    else
    {
      break;
    }
  }
  // 向前
  for(int step = 0;step<max_step_vec[test]+search_num/8;step++)
  {
    // 一系列的点
    Vector2i tempoint = frontVec[test] - step*frontToafterVec[test];
    
    if(!isOccupied(tempoint))
    {
      // 以当前点b为中心进行方块扩展
      Node_Extension(inflect_add_search[4*test+1],inflect_min_search[4*test+1],
      tempoint,verfrontToafterVec[test],front2Extension);
    }
    else
    {
      break;
    }
  }
  // 向后
  for(int step = 0;step<max_step_vec[test]+search_num/8;step++)
  {
    Vector2i tempoint = afterVec[test] + step*afterTofrontVec[test];

    // 以当前点b为中心进行方块扩展
    if(!isOccupied(tempoint))
    {
      // 以当前点b为中心进行方块扩展
      Node_Extension(inflect_add_search[4*test+2],inflect_min_search[4*test+2],
      tempoint,verfrontToafterVec[test],after1Extension);
    }
    else
    {
      break;
    }
  }

  for(int step = 0;step<max_step_vec[test]+search_num/8;step++)
  {
    Vector2i tempoint = afterVec[test] - step*afterTofrontVec[test];

    // 以当前点b为中心进行方块扩展
    if(!isOccupied(tempoint))
    {
      // 以当前点b为中心进行方块扩展
      Node_Extension(inflect_add_search[4*test+3],inflect_min_search[4*test+3],
      tempoint,verfrontToafterVec[test],after2Extension);
    }
    else
    {
      break;
    }
  }
}

void Fast_Security::Rectangle_Extension(int addsearch,int minsearch,Vector2i point,vector<Vector2i> &pointExtension)
{
  pointExtension.clear();
  // 水平向量
  Vector2i horizen = Vector2i(1,0);
  // 垂直向量
  Vector2i vertical = Vector2i(0,1);
  Vector2i tempoint;
  // 开始进行扩展
  for (int step = 0; step < addsearch; step++)
  {
    // 一系列的点
    Vector2i tempoint = point + step * horizen;
    if(!isOccupied(tempoint))
    {
      // 以当前点b为中心进行方块扩展
      Node_Extension(addsearch,minsearch,tempoint,vertical,pointExtension);
    }
    else
    {
      break;
    }
  }
  // 开始进行扩展
  for (int step = 1; step < minsearch; step++)
  {
    // 一系列的点
    Vector2i tempoint = point - step * horizen;
    if(!isOccupied(tempoint))
    {
      // 以当前点b为中心进行方块扩展
      Node_Extension(addsearch,minsearch,tempoint,vertical,pointExtension);
    }
    else
    {
      break;
    }
  }  
}

// 连续扩展部分提取
void Fast_Security::ContinuExtension_Extract(vector<vector<Vector2i>> &continu_sample)
{
  // 把中断连续点提取出来
  continu_sample.clear();
  // 开始对连续部分进行提取
  // 这里是每段的数量
  int start = 0;
  if(continuNum.size() >= 1)
  {
    for(int i=start;i<continuNum.size();i++)
    {
      // 提取终点
      int num = start + continuNum[i];
      // 分段提取
      continu_sample.push_back(continuPoints[start+continuNum[i]/8]);
      continu_sample.push_back(continuPoints[start+continuNum[i]/4]);
      continu_sample.push_back(continuPoints[num-continuNum[i]/4]);
      continu_sample.push_back(continuPoints[num-continuNum[i]/8]);
      // 计算每次的起点
      start+=continuNum[i];
    }
  }
  else
  {
    return;
  }
}

// 连续点扩展
// pointsMap 栅格地图返回的扩展点
// pointsWorld 世界地图返回的扩展点
void Fast_Security::Continu_Extension(vector<vector<Vector2i>> &pointsMap,vector<vector<Vector2d>> &pointsWorld)
{
  pointsMap.clear();
  pointsWorld.clear();

  // 统计持续段数目
  continuNum.clear();

  if(continuDivid.size()>0)
  {
    int num = 0;
    for (int i = 0; i < continuDivid.size()-1; i++)
    {
      if(continuDivid[i].first == continuDivid[i+1].first)
      {
        num++;
      }
      else
      {
        // 进行计数
        continuNum.push_back(num);
        num = 0;      
      }
    }

    // 对最后一段进行计数
    int thelastNum = 0;
    for (int i = 0; i < continuDivid.size(); i++)
    {
      if(continuDivid[i].first == flagOK[flagOK.size()-1])
      {
        thelastNum++;
      }
    }
    continuNum.push_back(thelastNum);

    // 查看最后一部分连续点
    // 获取最后一部分连续点数量
    int lastNum = continuNum[continuNum.size()-1];
    // 获取除了最后一段之前的总数
    int allNum = 0;
    for (int i = 0; i < continuNum.size()-1; i++)
    {
      /* code */
      // 获取持续点的数目
      allNum += continuNum[i];
    }

    // 获取最后一段的上下限制
    int last_add_num = 0;  //累加搜索点数目
    int last_min_num = 0;  //累减搜索点数目
    int last_add_search = search_num/2;
    int last_min_search = search_num/2;
    // 搜索最后一段的上下限
    for (int i = 0; i < lastNum; i++)
    {
      // 搜索节点限制
      Search_Obstacle(search_num,continuDivid[allNum+i].second,
      vercontinuDivid[allNum+i].second,last_add_num,last_min_num);
      // 计算当前段搜索的最小值
      if(last_add_num < last_add_search)
      {
        last_add_search = last_add_num;
      }
      if(last_min_num < last_min_search)
      {
        last_min_search = last_min_num;
      }
    }

    if(continuNum.size()>1)
    {
      int time = 0;
      // 连续的分段扩展
      for(int i=0;i<continuDivid.size()-1;i++)
      {
        int add_search = continu_add_search[time];
        int min_search = continu_min_search[time];
        
        // 6*0.05 = 0.3是车的最大尺寸半径
        // 如果小于6，则按照6扩展
        if(add_search < 6 && min_search < 6)
        {
          add_search = 6;
          min_search = 6;
        }

        vector<Vector2i> pointExtension;

        // 当i小于除了最后一段外的点的数目的时候
        if(i<allNum)
        {
          // 当是一段的时候进行扩展
          if(continuDivid[i].first == continuDivid[i+1].first)
          {
            // 开始进行扩展
            Node_Extension(add_search,min_search,continuDivid[i].second,vercontinuDivid[i].second,pointExtension);
            // 连续点访问点添加
            pointsMap.push_back(pointExtension);
            // 世界坐标系下的点集合
            pointsWorld.push_back(GridPathToWorldPath(pointExtension));

            vector<Vector2d> tempworld = GridPathToWorldPath(pointExtension);
          }
          else
          {
            time++;
            if(time >= continu_add_search.size())
            {
              time = continu_add_search.size()-1;
            }
          }
        }
        // 当大于的时候，最后一段添加进来
        else
        {
          // 开始进行扩展
          Node_Extension(last_add_search,last_min_search,continuDivid[i].second,vercontinuDivid[i].second,pointExtension);
          // 连续点访问点添加
          pointsMap.push_back(pointExtension);
          // 世界坐标系下的点集合
          pointsWorld.push_back(GridPathToWorldPath(pointExtension));

          vector<Vector2d> tempworld = GridPathToWorldPath(pointExtension);   
        }
      }
    }
  }
  else
  {

  }
}

// 中断区域扩展
// pointsMap 栅格地图返回的扩展点
// pointsWorld 世界地图返回的扩展点
void Fast_Security::Interrupt_Extension(vector<vector<Vector2i>> &pointsMap,vector<vector<Vector2d>> &pointsWorld)
{
  // 拐点搜索点集合清空
  pointsMap.clear();
  // 拐点集合清空
  pointsWorld.clear();

  // 前点集合清空
  frontVec.clear();
  // 后点集合清空
  afterVec.clear();

  // 前后向量集合清空
  frontToafterVec.clear();
  // 后前向量清空
  afterTofrontVec.clear();
  // 垂直于前后向量的集合清空
  verfrontToafterVec.clear();
  // 最大搜索步长清空
  max_step_vec.clear();

  inflect_add_search.clear();
  inflect_min_search.clear();

  // 当大于0的时候
  if(dividNodes.size() > 0)
  {
    // 打通不连通区域
    // 掐头去尾，中间两两一组
    for(int i=1;i<=dividNodes.size()-1;i++)
    {
      // 如果是十个点，顺序是0 1 2 3 4 5 6 7 8 9 
      // 0 12 34 56 78 9 其中在一块的是挨着的，我们要求的距离就是
      // 0-1 2-3 4-5 6-7 8-9 正好5段
      // cout<<5.21<<endl;
      int max_step;
      if(i%2 == 1)
      {
        // cout<<5.22<<endl;
        // 两两一组，求解向量进行扩展
        // 一组中前面的沿着向量方向-
        // 一组中后面的沿着向量方向+
        Vector2i front = dividNodes[i-1];
        Vector2i after = dividNodes[i];

        int distance_x = abs(front[0] - after[0]);
        int distance_y = abs(front[1] - after[1]);
        // 求解最大步长度
        max_step = max(distance_x,distance_y);
        
        // 添加进来
        max_step_vec.push_back(max_step);
      }
      // 计算间距
      //0 12 34 56 78 9分段  
      if(i%2 == 1 && i<=dividNodes.size()-2)
      {
        // cout<<5.23<<endl;
        // 两两一组，求解向量进行扩展
        // 一组中前面的沿着向量方向-
        // 一组中后面的沿着向量方向+
        Vector2i front = dividNodes[i];
        Vector2i after = dividNodes[i+1];

        frontVec.push_back(front);
        afterVec.push_back(after);

        // 前面的向量向前扩展，后面的向后扩展
        Vector2i frontToafter = front - after;
        // 向量单位化
        int distance_x = frontToafter[0];
        int distance_y = frontToafter[1];
        if(distance_x != 0 && distance_y != 0)
        {
          frontToafter[0] = distance_x / abs(distance_x);
          frontToafter[1] = distance_y / abs(distance_y);
        }
        if(distance_x == 0 && distance_y != 0)
        {
          frontToafter[0] = 0;
          frontToafter[1] = distance_y / abs(distance_y);
        }
        if(distance_x != 0 && distance_y == 0)
        {
          frontToafter[0] = distance_x / abs(distance_x);
          frontToafter[1] = 0;
        }

        Vector2i afterTofront = -frontToafter;

        // 垂直向量，因为两个向量共线段，所以只需要求解一次就行了
        Vector2i verfrontToafter = Vector2i(frontToafter[1],-frontToafter[0]);
        
        // 添加进来
        frontToafterVec.push_back(frontToafter);
        // 添加进来
        afterTofrontVec.push_back(afterTofront);
        // 垂直向量添加进来
        verfrontToafterVec.push_back(verfrontToafter);
      }
    }

    // 寻找中断部分的搜索上下限制
    for (int i = 0; i < frontToafterVec.size(); i++)
    {
      /* code */
      FindInflectLimitExten(i);
    }
    
    // // 这里存在问题，先注释掉，实验的时候更加方便
    // cout<<"frontToafterVec:"<<frontToafterVec.size()<<endl;
    if(frontToafterVec.size()>0)
    {
      for (int i = 0; i < frontToafterVec.size(); i++)
      {
        vector<Vector2i> front1Extension,after1Extension;
        vector<Vector2i> front2Extension,after2Extension;

        // 对中断的部分进行扩展
        Inflect_Extension(i,front1Extension,front2Extension,after1Extension,after2Extension);
        // 将扩展节点添加进来
        pointsMap.push_back(front1Extension);
        pointsMap.push_back(front2Extension);
        pointsMap.push_back(after1Extension);
        pointsMap.push_back(after2Extension);
        // 世界坐标点
        vector<Vector2d> worldPoints1 = GridPathToWorldPath(front1Extension);
        vector<Vector2d> worldPoints2 = GridPathToWorldPath(front2Extension); 
        vector<Vector2d> worldPoints3 = GridPathToWorldPath(after1Extension);  
        vector<Vector2d> worldPoints4 = GridPathToWorldPath(after2Extension); 
        // 中断点添加
        pointsWorld.push_back(worldPoints1);
        pointsWorld.push_back(worldPoints2);
        pointsWorld.push_back(worldPoints3);
        pointsWorld.push_back(worldPoints4);
      }
    }
  }
  else
  {
    // 拐点搜索点集合清空
    pointsMap.clear();
    // 拐点集合清空
    pointsWorld.clear();    
    
    return;
  }
}

// 设定地图参数
void Fast_Security::SetMapParams(double resolution_,double origin_x_,double origin_y_,
                    int map_x_size,int map_y_size,std::vector<int8_t> _mapData)
{
  resolution = resolution_;
  resolution_inv = 1.0/resolution;

  // 获得地图起点
  origin_x = origin_x_;
  origin_y = origin_y_;

  // 获得栅格地图尺寸
  grid_map_x = map_x_size;
  grid_map_y = map_y_size;

  mapover_x = grid_map_x*resolution;
  mapover_y = grid_map_y*resolution;

  // search_length = 0.25;
  // 这里设置的小一点
  search_num = search_length * resolution_inv;

	// 初始化一个数组,按照XYZ的大小去初始化数组
	mapdata = new int8_t[grid_map_x * grid_map_y]; // 为将地图转化为8进制栅格地图作准备
	// 内存处理,清空数组
	memset(mapdata, 0, grid_map_x * grid_map_y * sizeof(int8_t));

  // 设定地图
  for(int x=0;x<grid_map_x;x++){
    for(int y=0;y<grid_map_y;y++)
    {
      // 设定地图参数
      mapdata[x+y*grid_map_x] = _mapData[x+y*grid_map_x];
    }
  }

  // 和栅格地图尺寸一样大的数组
  search_data.resize(grid_map_x*grid_map_y);
  // 将地图全变为0
  search_data.assign(grid_map_x*grid_map_y,0);
  
  bezier_opt.reset(new BEZIER);
  // 贝塞尔优化
  bezier_opt->setParams(3.0,0.2,4);
}

// 路径点采样
void Fast_Security::SampleGridPath(vector<Vector2i> gridpath)
{
  int path_node_num = gridpath.size();

  int inflectnum = 0; //中断点个数
  int continunum = 0; //中断点个数

  int dividNum = 1;

  inflectAssem.clear();

  // 定义一个采样窗口 每次采样两个
  for(int i=1;i<path_node_num-1;i++)
  {
    // 因为Astar规划的路径拐角是固定的,所以这里直接计算了,上一个点减去下一个点,因为八连通astar探索点就是八个方向,相当于按照方向扩展
    pointa = gridpath[i-1];
    pointb = gridpath[i];

    // 求向量AB
    v_b_a = pointb - pointa; 
    // 求解与向量AB垂直的向量
    ver_b_a = Vector2i(v_b_a[1],-v_b_a[0]);

    // 当前向量
    nowvec = ver_b_a;

    // 判断是否共线
    if(!isCollinear(nowvec,lastvec,true))
    {
      // cout<<"i-inflectnum: "<<i-inflectnum<<endl;
      // 中断点个数
      inflectnum = i;
      // 不共线向量点集合
      inflectAssem.push_back(pointb);
    }
    // 共线就添加到共线中
    else
    {
      // cout<<"i-continunum: "<<i-continunum<<endl;
      if(i-continunum == 1)
      {
        // 连续共线的点添加
        continuAssem.push_back(pointb);
        // 垂直向量添加
        vercontAssem.push_back(ver_b_a);

        // 开始分段
        continuDivid.push_back(make_pair(dividNum,pointb));
        // 垂直向量也进行分段
        vercontinuDivid.push_back(make_pair(dividNum,ver_b_a));
      }
      else
      {
        dividNum ++;
      }
      // 连续点个数
      continunum = i;
    }
    // 上一个向量
    lastvec = nowvec;
  }

#if 0
  vector<double> anglevec1;
  // 计算向量方法提取采样点时间
  ros::Time time_1 = ros::Time::now();
  for (int i = 1; i < path_node_num-1; i++)
  {
    // 上一个点
    Vector2d lastpoint = originPath[i-1];
    Vector2d nowpoint = originPath[i];
    Vector2d nextpoint = originPath[i+1];

    // 求解中点
    Vector2d medpoint = 0.5*(lastpoint+nextpoint);
    // 连接中点和当前点的连线 形成向量
    Vector2d med_nowpoint = nowpoint-medpoint;
    // 求解垂直向量
    Vector2d ver_med_now;

    if(nowpoint != medpoint)
    {
      // 求解垂直向量的角度朝向
      ver_med_now = Vector2d(med_nowpoint[1],-med_nowpoint[0]);
      // 向量单位化
      ver_med_now = calUnitvector(ver_med_now);
    }
    if(nowpoint == medpoint)
    {
      ver_med_now = nextpoint - lastpoint;
      ver_med_now = calUnitvector(ver_med_now);
    }
    // 当前切线数值
    double angle = acos(ver_med_now[0]);

    // 输出角度
    anglevec1.push_back(angle);
  } 
  ros::Time time_2 = ros::Time::now();
  ROS_WARN("sample path time-向量:%f",(time_2-time_1).toSec() * 1000.0);

  vector<double> anglevec2;
  // 计算向量方法提取采样点时间
  ros::Time time_3 = ros::Time::now();
  for (int i = 1; i < path_node_num-1; i++)
  {
      // 将路径点转换成矩阵
    MatrixXd xy(3,2);
    // 对xy赋值
    xy(0,0) = originPath[i-1][0];
    xy(0,1) = originPath[i-1][1];
    xy(1,0) = originPath[i][0];
    xy(1,1) = originPath[i][1];
    xy(2,0) = originPath[i+1][0];
    xy(2,1) = originPath[i+1][1];

    // 对d_B赋值
    double d_B = curve.calDerivative(xy,xy(1,0));
    // 
    double angle = atan(d_B);
    // 
    anglevec2.push_back(angle);
  }
  ros::Time time_4 = ros::Time::now();
  ROS_WARN("sample path time-求导:%f",(time_4-time_3).toSec() * 1000.0);  

  // file.open("/home/lilei/Robot_Demo/MyFormProject/src/robot_navigation/test.txt",ios::app);
  // if(file.is_open())
  // {
  //   // 
  //   cout<<"文件已创建"<<endl;
  //   for (int i = 0; i < anglevec1.size(); i++)
  //   {
  //     // cout<<"  "<<anglevec1[i]<<"   "<<anglevec2[i]<<endl;
  //     file<<i<<"  "<<anglevec1[i]<<"  "<<anglevec2[i]<<endl;
  //   }
  //   file.close();
  // }
#endif

  // 对分段后的点进行处理删除调连续数目<search_num/4的点
  vector<pair<int,Vector2i>> tempcontinDiv = continuDivid;
  vector<pair<int,Vector2i>> vertempcontinDiv = vercontinuDivid;
  // 先清空一下
  continuDivid.clear();
  vercontinuDivid.clear();

  flagOK.clear();
  bool hasAdd = false;
  int time = 0;
  // 连续部分分段数字
  int continuDivNum = search_num/4;
  // 连续段数目判断
  if(continuDivNum > 5)
  {
    continuDivNum = 5;
  }
  for (int i = 0; i < tempcontinDiv.size()-1; i++)
  {
    if(tempcontinDiv[i].first == tempcontinDiv[i+1].first)
    {
      time++;
      if(time >= continuDivNum && hasAdd == false)
      {
        hasAdd = true;
        // 添加进来
        flagOK.push_back(tempcontinDiv[i].first);
      }
    }
    else
    {
      // 当大于continuDivNum的时候
      time = 0;
      // 
      hasAdd = false;
    }

  }
  
  // 输出大于continuDivNum段的个数
  // // 将连续的点数目>continuDivNum的段添加进来
  for (int num = 0; num < flagOK.size(); num++)
  {
    for (int i = 0; i < tempcontinDiv.size(); i++)
    {
      if(tempcontinDiv[i].first == flagOK[num])
      {
        // 添加进来
        continuDivid.push_back(tempcontinDiv[i]);
        // 垂直向量也添加进来
        vercontinuDivid.push_back(vertempcontinDiv[i]);
      }
    }
  }

}

// 搜索扩展限制点
// add_search_vec 向上扩展容器
// min_search_vec 向下扩展容器
void Fast_Security::FindContinueLimitExten(vector<int> &add_search_vec,vector<int> &min_search_vec)
{
  // 限制清0
  add_search_vec.clear();
  min_search_vec.clear();

  int search_add_num = 0;  //累加搜索点数目
  int search_min_num = 0;  //累减搜索点数目
  int add_search;
  int min_search;

  if(continuDivid.size()>0)
  {
    // 对连续点进行扩展搜索上下限
    // 分段计算
    for(int i=0;i<continuDivid.size()-1;i++)
    {
      // 当是一段的时候，计算一次最小值
      if(continuDivid[i].first == continuDivid[i+1].first)
      {
        // 连续节点扩展 搜索每段上探索的两侧最小值
        Search_Obstacle(search_num,continuDivid[i].second,vercontinuDivid[i].second,search_add_num,search_min_num);
        // 计算当前段搜索的最小值
        if(search_add_num < add_search)
        {
          add_search = search_add_num;
        }
        if(search_min_num < min_search)
        {
          min_search = search_min_num;
        }
      }
      // 不是的时候，保存当前计算出来的最小值
      else
      {
        // 添加进来
        add_search_vec.push_back(add_search);
        min_search_vec.push_back(min_search);

        // 这里不清0会出现问题，每次得出上下限之后一定要清0
        add_search = search_num/2;
        min_search = search_num/2;

        search_add_num = 0;
        search_min_num = 0;
      }
    }
  }
  else
  {
    // 限制清0
    add_search_vec.clear();
    min_search_vec.clear();    
  }
}

// 搜索扩展限制点
// add_search_vec 向上扩展容器
// min_search_vec 向下扩展容器
void Fast_Security::FindInflectLimitExten(int test)
{
  // 对中断处的连续部分扩展，搜索上下限
  int search_add_num = 0;  //累加搜索点数目
  int search_min_num = 0;  //累减搜索点数目
  int add_search = search_num/2;
  int min_search = search_num/2;

  // 对连续点进行扩展搜索上下限
  // 分段计算
  for(int step = 0;step<search_num/2;step++)
  {
    // 一系列的点
    Vector2i tempoint = frontVec[test] + step*frontToafterVec[test];
    // 连续节点扩展 搜索每段上探索的两侧最小值
    Search_Obstacle(search_num,tempoint,verfrontToafterVec[test],search_add_num,search_min_num);
    // 计算当前段搜索的最小值
    if(search_add_num < add_search)
    {
      add_search = search_add_num;
    }
    if(search_min_num < min_search)
    {
      min_search = search_min_num;
    }
  }

  // 添加进来
  inflect_add_search.push_back(add_search);
  inflect_min_search.push_back(min_search);

  // 这里不清0会出现问题，每次得出上下限之后一定要清0
  add_search = search_num/2;
  min_search = search_num/2;

  search_add_num = 0;
  search_min_num = 0;


  // 对连续点进行扩展搜索上下限
  // 分段计算
  for(int step = 0;step<search_num/2;step++)
  {
    // 一系列的点
    Vector2i tempoint = frontVec[test] - step*frontToafterVec[test];
    // 连续节点扩展 搜索每段上探索的两侧最小值
    Search_Obstacle(search_num,tempoint,verfrontToafterVec[test],search_add_num,search_min_num);
    // 计算当前段搜索的最小值
    if(search_add_num < add_search)
    {
      add_search = search_add_num;
    }
    if(search_min_num < min_search)
    {
      min_search = search_min_num;
    }
  }

  // 添加进来
  inflect_add_search.push_back(add_search);
  inflect_min_search.push_back(min_search);

  // 这里不清0会出现问题，每次得出上下限之后一定要清0
  add_search = search_num/2;
  min_search = search_num/2;

  search_add_num = 0;
  search_min_num = 0;

  // 对连续点进行扩展搜索上下限
  // 分段计算
  for(int step = 0;step<search_num/2;step++)
  {
    // 一系列的点
    Vector2i tempoint = afterVec[test] + step*afterTofrontVec[test];
    // 连续节点扩展 搜索每段上探索的两侧最小值
    Search_Obstacle(search_num,tempoint,verfrontToafterVec[test],search_add_num,search_min_num);
    // 计算当前段搜索的最小值
    if(search_add_num < add_search)
    {
      add_search = search_add_num;
    }
    if(search_min_num < min_search)
    {
      min_search = search_min_num;
    }
  }

  // 添加进来
  inflect_add_search.push_back(add_search);
  inflect_min_search.push_back(min_search);

  // 这里不清0会出现问题，每次得出上下限之后一定要清0
  add_search = search_num/2;
  min_search = search_num/2;

  search_add_num = 0;
  search_min_num = 0;

  // 对连续点进行扩展搜索上下限
  // 分段计算
  for(int step = 0;step<search_num/2;step++)
  {
    // 一系列的点
    Vector2i tempoint = afterVec[test] - step*afterTofrontVec[test];
    // 连续节点扩展 搜索每段上探索的两侧最小值
    Search_Obstacle(search_num,tempoint,verfrontToafterVec[test],search_add_num,search_min_num);
    // 计算当前段搜索的最小值
    if(search_add_num < add_search)
    {
      add_search = search_add_num;
    }
    if(search_min_num < min_search)
    {
      min_search = search_min_num;
    }
  }

  // 添加进来
  inflect_add_search.push_back(add_search);
  inflect_min_search.push_back(min_search);

  // 这里不清0会出现问题，每次得出上下限之后一定要清0
  add_search = search_num/2;
  min_search = search_num/2;

  search_add_num = 0;
  search_min_num = 0;  

  // 当膨胀点数目<search_num/8的时候,以search_num/8进行膨胀
  // 当膨胀点数目<6的时候,以6进行膨胀
  for (int i = 0; i < inflect_add_search.size(); i++)
  {
    /* code */
    if(inflect_add_search[i] < search_num/4 && inflect_min_search[i] < search_num/4)
    {
      inflect_add_search[i] = search_num/4;
      inflect_min_search[i] = search_num/4;
    }
  }
  
}

// 计算分段点
// dividMap 栅格地图分段点
// dividWorld 世界地图分段点
void Fast_Security::CalDivideNodes(vector<Vector2i> &dividMap,vector<Vector2d> &dividWorld)
{
  // 定义分段点
  dividMap.clear();
  dividWorld.clear();

  // // 这里存在问题，但是为了实验方面，先注释掉
  // cout<<"continuDivid:"<<continuDivid.size()<<endl;

  if(continuDivid.size()>0)
  {
    // 用于显示分段点的
    vector<Vector2i> frontpoints;
    // 向前显示分段点 ---x---a---
    for(int i = 0;i<continuDivid.size()-1;i++)
    {
      if(continuDivid[i].first != continuDivid[i+1].first)
      {
        frontpoints.push_back(continuDivid[i].second);
      }
    }
    // 用于显示分段点的---a---x---
    vector<Vector2i> afterpoints;
    // 向后显示分段点---a---x---
    for(int i = 1;i<continuDivid.size();i++)
    {
      if(continuDivid[i].first != continuDivid[i-1].first)
      {
        afterpoints.push_back(continuDivid[i].second);
      }
    }

    // 添加分段点
    for (int i = 0; i < afterpoints.size(); i++)
    {
      dividMap.push_back(frontpoints[i]);
      dividMap.push_back(afterpoints[i]);

      dividWorld.push_back(mapToWorld(frontpoints[i]));
      dividWorld.push_back(mapToWorld(afterpoints[i]));
    }
  }
  else
  {
    // 定义分段点
    dividMap.clear();
    dividWorld.clear();    
  }
}

vector<Vector2d> Fast_Security::Fast_Security_Search(vector<Vector2d> oripath)
{
  completed_flag = 0;
  // 清除所有用到的容器
  resetVisited();

  // 将地图全变为0
  // 如果不归零无法正确判断重复访问的区域
  search_data.assign(grid_map_x*grid_map_y,0);
  
  // 先对输入路径的节点数目进行判断，因为三角形至少需要三个组成点
  int path_node_num = oripath.size();
  vector<Vector2i> gridpath = worldPathToGridPath(oripath);
  origingridPath = gridpath;

  // 当搜索路径点数目<3的时候，即只有起点和终点在
  if(path_node_num < 3)
  {
    return oripath; //返回原来路径，不做优化
  }

  // 当搜索路径点数目 >= 3的时候开始进行优化
  else{
    // 
    grid_map_start = gridpath[0];
    grid_map_end = gridpath[path_node_num-1];

    // 对路径点进行扩展
    SampleGridPath(gridpath);

    // 求解分段点
    CalDivideNodes(dividNodes,worldividNodes);

    // 搜索扩展限制
    FindContinueLimitExten(continu_add_search,continu_min_search);

    // 连续区域扩展
    Continu_Extension(continuPoints,continuPointsWorld);
    // 从连续扩展部分中进行提取
    ContinuExtension_Extract(continuSample);

    // 中断区域扩展
    Interrupt_Extension(inflectPoins,inflectPoinsWorld);

    // 对路径进行优化
    optedpath = PathOptimization(continuSample,inflectPoins);
  }

  return optedpath;
}

void Fast_Security::visualNodes(void)
{
  // 查看所有有用访问点的探索情况
  // visual_SamplesNode(continuNodesPub,continuPointsWorld,0.5,0,0,1,1);
  // visual_SamplesNode(visitNodesPub,inflectPoinsWorld,0.5,1,0,0,1);
  // visual_SamplesNode(sampleNodesPub,connectPointsWorld,1,0,1,1,1);
  // visual_VisitedNode(visitOptpathPub,visualOptpoints,1,1,0,0,1);

  visual_SamplesNode(continuNodesPub,continuPointsWorld,0.5,1,0,0,1);
  visual_SamplesNode(visitNodesPub,inflectPoinsWorld,0.5,1,0,0,1);
  visual_SamplesNode(sampleNodesPub,connectPointsWorld,0.5,1,0,0,1);
  visual_VisitedNode(visitOptpathPub,optedpath,0.5,1,0,0,1);
}

// 查询间断点的数目
// 这里的间断点是中断扩展和连续扩展都没包含的点
// gridPath 栅格路径点
// firstpoint 输入的第一个点
// secondpoint 输入的第二个点
// firstnum 输出的第一个点的位置
// secondnum 输出的第二个点的位置
void Fast_Security::CheckInflectNum(vector<Vector2i> gridPath,Vector2i firstpoint,Vector2i secondpoint,int &firstnum,int &secondnum)
{
  for (int length = 0; length < gridPath.size(); length++)
  {
    // 开始在原始路径上搜索
    if(gridPath[length] == firstpoint)
    {
      firstnum = length;
    }
    if(gridPath[length] == secondpoint)
    {
      secondnum = length;
    }
  }
}

// 输出路径点前后共线段的坐标位置集合
// gridPath 栅格路径点
vector<int> Fast_Security::CollinearPosition(vector<Vector2i> gridPath)
{
  // 用来保存共线的断点在opt中的位置
  vector<int> collinear_Num; 
  // 对路径进行动态扫描，根据中间点判断前后两个点是否共线，共线就分段添加
  for (int i = 1; i < gridPath.size()-1; i++)
  {
    // 按照前后向量判断是否共线段
    Vector2i lastpoint = gridPath[i-1];
    Vector2i nowpoint = gridPath[i];
    Vector2i nextpoint = gridPath[i+1];

    // 定义之前和现在的点形成的向量
    Vector2i last_now = nowpoint-lastpoint;
    // 定义现在和之后的点形成的向量
    Vector2i now_next = nextpoint - nowpoint;

    // 如果相等记录当前i数值，并把之前的点添加进来
    // 当共线的时候
    if(isCollinear(last_now,now_next,true))
    {
      // 开始添加对应的点在路径中的位置
      collinear_Num.push_back(i);
    }
  }

  return collinear_Num;
}

// 对路径进行分段
// gridPath 栅格路径点
// dividvec 用于分段的数组
// dividpath 分段出来的栅格路径点
void Fast_Security::DividPath(vector<Vector2i> gridPath,vector<int> dividvec,vector<vector<Vector2d>> &dividpath)
{
  dividpath.clear();  

  // 开始分段
  for (int i = 0; i < dividvec.size(); i++)
  {
    if(i<dividvec.size()-1)
    {
      // 开始点
      int start = dividvec[i];
      // 结束点
      int end = dividvec[i+1]+1;
      if(i == dividvec.size()-2)
      {
        end = dividvec[i+1];
      }
      if(i == 0)
      {
        // 进行添加
        vector<Vector2i> tempath;
        // 根据当前的位置进行添加
        for (int j = 0; j < start; j++)
        {
          // 把点添加进来
          tempath.push_back(gridPath[j]);
        }
        dividpath.push_back(GridPathToWorldPath(tempath));
      }
      if(i > 0)
      {
        // 进行添加
        vector<Vector2i> tempath;
        // 根据当前的位置进行添加
        for (int j = start; j < end; j++)
        {
          // 把点添加进来
          tempath.push_back(gridPath[j]);
        }
        dividpath.push_back(GridPathToWorldPath(tempath));
      }
    }
    // 把最后一段放进来
    if(i==dividvec.size()-1)
    {
      // 进行添加
      vector<Vector2i> tempath;
      // 根据当前的位置进行添加
      for (int j = dividvec[i]; j < gridPath.size(); j++)
      {
        // 把点添加进来
        tempath.push_back(gridPath[j]);
      }
      dividpath.push_back(GridPathToWorldPath(tempath));
    }
  }
}

// 使用贝塞尔曲线对路径进行优化
vector<Vector2d> Fast_Security::BezierPathOpt(vector<vector<Vector2d>> needoptpath)
{
  vector<Vector2d> worldpath;
  vector<vector<pair<int,Vector2d>>> waitopt;

  // 对等待初始化的路径进行初始化
  for (int i = 0; i < needoptpath.size(); i++)
  {
    vector<pair<int,Vector2d>> sub;
    for (int j = 0; j < needoptpath[i].size(); j++)
    {
      /* code */
      // 设定为0 代表当前点没有访问过
      sub.push_back(make_pair(0,needoptpath[i][j]));
    }
    waitopt.push_back(sub);    
  }

  // 使用贝塞尔进行分段优化
  for (int i = 0; i < needoptpath.size(); i++)
  {
    // 当路径点数目大于4的时候，进行筛选
    vector<Vector2d> dividopt = needoptpath[i];
    int dividlength = dividopt.size();
    // 直接开始优化
    if(dividlength <= 4)
    {
      //  开始优化
      vector<Vector2d> newoptpath = bezier_opt->BezierPath(dividopt);
      for (int num = 0; num < newoptpath.size(); num++)
      {
        // 添加进来
        worldpath.push_back(newoptpath[num]);
      }
    }
    // 如果大于4的时候
    if(dividlength > 4)
    {      
      for (int num = 0; num < dividlength/4; num++)
      {
        /* code */
        vector<Vector2d> tempath;
        // 取四个点
        tempath.push_back(dividopt[num*4]);
        tempath.push_back(dividopt[num*4+1]);
        tempath.push_back((dividopt[num*4+2]));
        tempath.push_back(dividopt[num*4+3]);
        //  开始优化
        vector<Vector2d> newoptpath = bezier_opt->BezierPath(tempath);        
        // 
        for (int length = 0; length < newoptpath.size(); length++)
        {
          // 添加进来
          worldpath.push_back(newoptpath[length]);
        }        
      }
      // 将最后一段放进来
      int lastlength = dividlength%4;
      // 当大于0的时候
      if(lastlength > 0)
      {
        vector<Vector2d> tempath;
        // 开始累加
        for (int num = 0; num < lastlength; num++)
        {
          tempath.push_back(dividopt[4*(dividlength/4)+num]);
        }

        //  开始优化
        vector<Vector2d> newoptpath = bezier_opt->BezierPath(tempath);
        // 
        for (int length = 0; length < newoptpath.size(); length++)
        {
          // 添加进来
          worldpath.push_back(newoptpath[length]);
        }         
      }
    }
  }

  // 转换出来
  vector<Vector2i> tempath = worldPathToGridPath(worldpath);
  // // 插入路径
  vector<Vector2i> interpo_path;
  // // 对路径再次进行插值
  Setlength_Interpolation_Path(0.2,tempath,interpo_path);
  // 输出插值后的路径
  vector<Vector2d> lastpath = GridPathToWorldPath(interpo_path);
  
  vector<Vector2d> optedpath;
  // 上一个点
  Vector2d lastpoint;
  // 每隔0.2一个点开始采样
  for (int i = 0; i < lastpath.size(); i++)
  {
    if(i == 0)
    {
      optedpath.push_back(lastpath[0]);
      lastpoint = lastpath[0];
    }
    if(i>0)
    {
      // 计算距离
      double distance = calPointLength(lastpath[i],lastpoint);
      if(distance >= 0.2)
      {
        // 添加进来
        optedpath.push_back(lastpath[i]);
        // 传参
        lastpoint = lastpath[i];
      }
    }
    if(i == lastpath.size()-1)
    {
      // 添加进来
      optedpath.push_back(lastpath[i]);      
    }
  }
  
  return optedpath;
}

// 获取优化后的路径的欧几里得距离场信息数据
void Fast_Security::GetPathESDFvalue(int &sum_esdf,double &ave_esdf,int &path_num)
{
  sum_esdf = 0;
  ave_esdf = 0;
  path_num = 0;
  // 获取优化路径点数目
  int optedpath_num = optedpath.size();

  // cout<<"optedpath_num:"<<optedpath_num<<endl;
  // 
  if(optedpath_num > 0)
  {
    // 按照地图分辨率进行采样
    Vector2d nowpoint = optedpath[0];
    for (int i = 0; i < optedpath_num; i++)
    {
      // 获取当前点
      if(calPointLength(nowpoint,optedpath[i]) >= resolution)
      {
        path_num ++;
        sum_esdf += getESDFvalue(worldToMap(optedpath[i]));
        nowpoint = optedpath[i];
      }
    }
    
    ave_esdf = sum_esdf/(1.0f*path_num);
  }
  else
  {
    return;
  }
}

// 检查点之间有没有障碍物
// firstpoint  第一个点
// secondpoint 第二个点
bool Fast_Security::CheckObstaclePoints(Vector2i firstpoint,Vector2i secondpoint)
{
  bool has_obstacle = false;
  // 按照向量的形式进行检测查看是否有障碍物
  double length = calPointLength(firstpoint,secondpoint);
  // 定义方向
  Vector2i direct = secondpoint-firstpoint;
  Vector2d direct_unit = calUnitvector(direct);
  // 判断有没有障碍物
  for (int step = 0; step < length; step++)
  {
    // 搜索的点
    Vector2d searchpoint = mapToWorld(firstpoint) + step*direct_unit*resolution;
    // 是否碰到障碍物
    if(isOccupied(searchpoint))
    {
      // cout<<"有障碍物"<<endl;
      has_obstacle = true;
      break;
    }
  }

  return has_obstacle;
}

// 对路径按照固定间距分段
// step 分段长度
// oldpath 原始路径
// dividpath 分段之后的路径
void Fast_Security::Setlength_DividPath(double step,vector<Vector2i> oldpath,vector<Vector2i> &newpath)
{
  // 上一个点
  Vector2i lastpoint;
  // 每隔0.2一个点开始采样
  for (int i = 0; i < oldpath.size(); i++)
  {
    if(i == 0)
    {
      newpath.push_back(oldpath[0]);
      lastpoint = oldpath[0];
    }
    if(i>0)
    {
      // 计算距离
      double distance = calPointLength(mapToWorld(oldpath[i]),mapToWorld(lastpoint));
      if(distance >= 0.2)
      {
        // 添加进来
        newpath.push_back(oldpath[i]);
        // 传参
        lastpoint = oldpath[i];
      }
    }
    if(i == oldpath.size()-1)
    {
      // 添加进来
      newpath.push_back(oldpath[i]);      
    }
  }
}

// 对路径进行插值，这里是按照距离等距插值
// length 插值距离
// oldpath 原来路径
// interpolatpath 插值路径
void Fast_Security::Setlength_Interpolation_Path(double length,vector<Vector2i> oldpath,vector<Vector2i> &newpath)
{
  double steplength = length;
  double interpolation_length = resolution;

  newpath.clear();
  for (int i = 0; i < oldpath.size()-1; i++)
  {
    // 当前点
    Vector2i nowpoint = oldpath[i];
    Vector2i nextpoint = oldpath[i+1];
    // 计算两点之间的距离
    double distance = calPointLength(nowpoint,nextpoint)*resolution;

    // 步长大于设定值的时候进行插值
    if(distance > steplength)
    {
      // 计算向量
      Vector2i now_next = nextpoint-nowpoint;
      // 求解单位向量
      Vector2d unit_now_next = calUnitvector(now_next);

      newpath.push_back(oldpath[i]);
      // 进行插值
      int step_num = distance/interpolation_length; //计算插值数目，或者说插值的步长
      // 开始插值
      for (int j = 1; j < step_num; j++)
      {
        // 插值点
        Vector2d startpoint = mapToWorld(nowpoint);
        Vector2d interpolationpoint = startpoint+j*unit_now_next*resolution;
        newpath.push_back(worldToMap(interpolationpoint));
      }
    }
    else
    {
      newpath.push_back(oldpath[i]);
    }
  }
}

// 对路径进行优化
vector<Vector2d> Fast_Security::PathOptimization(vector<vector<Vector2i>> continu,vector<vector<Vector2i>> inflect)
{
  vector<Vector2d> worldpath;
  vector<Vector2i> optpath;
  vector<pair<int,Vector2i>> samplePoints;

  vector<pair<Vector2i,Vector2i>> sampleSortPoints;

  optimiPoints.clear();
  optimiPointsWorld.clear();
  connectPointsWorld.clear();

  // 将中间点进行插入 inflectPoins 从1开始算 12前插入一段 34前插入一段
  for (int i = 0; i < continu.size(); i++)
  {
    if(continu[i].size() > 0)
    {
      // 被2整除的时候进行添加
      // 添加连续部分的点
      // 不为空的时候添加进来
      if(continu[i].size() > 0)
      {
        optimiPoints.push_back(make_pair(1,continu[i]));
        optimiPointsWorld.push_back(GridPathToWorldPath(continu[i]));
      }
    }

    if(i%4 == 0 && i/2+1 < dividNodes.size())
    {
      // 有间断的时候进行补充扩展
      // 当i<continu.size()-1的时候，对间断点扫描
      if(i < continu.size()-1)
      {
        Vector2i first = dividNodes[i/2];
        Vector2i second = dividNodes[i/2+1];
        // 搜索first 和 second中间未被扩展过的节点
        int firstNum = 0;
        int secondNum = 0;
        // 检查中断点数目
        CheckInflectNum(origingridPath,first,second,firstNum,secondNum);
        // 必须大于1的时候，小的断点直接滤波去掉
        if(secondNum - firstNum > 3)
        {
          // 判断斜率
          Vector2i first_second = origingridPath[secondNum] - origingridPath[firstNum];
          // 根据斜率判断朝向
          Vector2i ver_first_second;

          // 将向量标准化，这里的标准化是标准化为八个方向
          if(first_second[0] == 0 && first_second[1] > 0)
          {
            ver_first_second = Vector2i(1,0);
          }
          if(first_second[0] > 0 && first_second[1] == 0)
          {
            ver_first_second = Vector2i(0,1);
          }
          if(first_second[0] * first_second[1] > 0)
          {
            ver_first_second = Vector2i(1,-1);
          }
          if(first_second[0] * first_second[1] < 0)
          {
            ver_first_second = Vector2i(1,1);
          }
          // 开始判断是否扩展过,进行扩展
          for (int start = firstNum; start < secondNum; start++)
          {
            vector<Vector2i> pointExtension;
            // 开始扩展 没扩展到的点进行扩展
            if(getVisitedNum(origingridPath[start]) == 0)
            {
              Node_Extension(search_num/2,search_num/2,origingridPath[start],ver_first_second,pointExtension);
              // 添加进来
              optimiPoints.push_back(make_pair(3,pointExtension));       

              optimiPointsWorld.push_back(GridPathToWorldPath(pointExtension));
              // 连接处扩展节点添加
              connectPointsWorld.push_back(GridPathToWorldPath(pointExtension));
            }
          }
        }
      }
    }

    // 将中断部分添加进来
    if(i%4 == 1 && (i-1)*4+3 < inflect.size())
    {
      if(inflect[(i-1)*4].size() > 0)
      {
        optimiPoints.push_back(make_pair(2,inflect[(i-1)*4]));
        optimiPointsWorld.push_back(GridPathToWorldPath(inflect[(i-1)*4]));
      }
      if(inflect[(i-1)*4+1].size() > 0)
      {
        optimiPoints.push_back(make_pair(2,inflect[(i-1)*4+1]));
        optimiPointsWorld.push_back(GridPathToWorldPath(inflect[(i-1)*4+1]));
      }
      if(inflect[(i-1)*4+2].size() > 0)
      {
        optimiPoints.push_back(make_pair(2,inflect[(i-1)*4+2]));
        optimiPointsWorld.push_back(GridPathToWorldPath(inflect[(i-1)*4+2]));
      }
      if(inflect[(i-1)*4+3].size() > 0)
      {
        optimiPoints.push_back(make_pair(2,inflect[(i-1)*4+3]));
        optimiPointsWorld.push_back(GridPathToWorldPath(inflect[(i-1)*4+3]));
      }
    }
  }

  // 计算最大距离
  double max_distance = calPointLength(grid_map_start,grid_map_end);
  // 对optimiPoints进行插值,将supplement插入到对应位置
  // 获取到访问的点,按照顺序,然后把为访问的节点插入进来
  // 方法对optimiPoints进行搜索,查看其中访问到的origin_path的点,没有的就把补充部分插入进来
  // 二次采样，查看所有重复节点
  Vector2i last;
  for (int i = 0; i < optimiPoints.size(); i++)
  {
    // 输出每个部分的最小值
    pair<int,vector<Vector2i>> tempoints = optimiPoints[i];
    // 当是连续部分的时候
    if(tempoints.first == 1)
    {
      double nowvisit;
      double min_cost = 10000;
      double cost;
      int8_t now_esdf;

      Vector2i nowpoint;
      Vector2i tempoint;
      double last_distance = calPointLength(last,tempoints.second[0]);
      double end_distance = calPointLength(grid_map_end,tempoints.second[0]);
      // 参考代价数值
      double refer_esdf = getESDFvalue(tempoints.second[0]);
      // 参考代价数值所占比例，代价越高就证明距离障碍物近，则削弱到终点距离和到上一个点距离代价占比
      double dangerous = refer_esdf/100.0f;   //越大越危险
      // 遍历搜索势场最小点数值
      for(int t=0;t<tempoints.second.size();t++)
      {
        // 获取势场数值
        nowpoint = tempoints.second[t];
        nowvisit = getVisitedNum(nowpoint);
        now_esdf = getESDFvalue(nowpoint);
        double distance = calPointLength(nowpoint,last);
        double dis_end = calPointLength(nowpoint,grid_map_end);
        // 定义系数，距离终点约近，则削弱势场所占比例
        double esdf_edis = dis_end/max_distance;
        esdf_edis = LIMIT_VALUE(esdf_edis,0.5,1.0);
        // 计算总的代价值，势场代价，到上一个点的距离代价，到终点的距离代价
        cost = (esdf_edis)*Kcontinu_esdf*now_esdf/(refer_esdf+1.0) + (1-dangerous)*Kcontinu_ldis*distance/(last_distance+1.0) 
        + (1-dangerous)*Kcontinu_edis*dis_end/(end_distance+1.0);
        if(cost < min_cost)
        {
          min_cost = cost;
          tempoint = nowpoint;
        }
      }
      last = tempoint;
      // 测试节点添加进来用于排序
      sampleSortPoints.push_back(make_pair(tempoints.second[0],tempoint));
    }
    // 当是2的时候 
    if(tempoints.first == 2)
    {
      // int min_visit = 1;
      double nowvisit;
      double min_cost = 10000;
      double cost;
      int8_t now_esdf;

      Vector2i nowpoint;
      Vector2i tempoint;
      double last_distance = calPointLength(last,tempoints.second[0]);
      double end_distance = calPointLength(grid_map_end,tempoints.second[0]);
      // 参考代价数值
      double refer_esdf = getESDFvalue(tempoints.second[0]);
      double dangerous = refer_esdf/100.0f;   //越大越危险
      // 遍历搜索势场最小点数值
      for(int t=0;t<tempoints.second.size();t++)
      {
        // 获取势场数值
        nowpoint = tempoints.second[t];
        nowvisit = getVisitedNum(nowpoint);
        now_esdf = getESDFvalue(nowpoint);
        // cost = now_esdf/100.0f + 1.0f/nowvisit;
        // cost = now_esdf/100.0f;
        double dis_end = calPointLength(nowpoint,grid_map_end);
        double distance = calPointLength(nowpoint,last);
        // 定义系数，距离终点约近，则削弱势场所占比例
        double esdf_edis = dis_end/max_distance;
        esdf_edis = LIMIT_VALUE(esdf_edis,0.5,1.0);
        // 计算总的代价值，势场代价，到上一个点的距离代价，到终点的距离代价，和重复点代价
        cost = (esdf_edis)*esdf_edis*Kinterru_esdf*now_esdf/(refer_esdf+1.0) + (1-dangerous)*Kinterru_rep*1.0f/nowvisit + 
        Kinterru_ldis*distance/(last_distance+1.0) + (1-dangerous)*Kinterru_edis*dis_end/(end_distance+1.0);
        if(cost < min_cost && nowvisit > 1)
        {
          min_cost = cost;
          tempoint = nowpoint;
        }
      }
      last = tempoint;
      // 测试节点添加进来用于排序
      sampleSortPoints.push_back(make_pair(tempoints.second[0],tempoint));
    }
    // 当是3的时候 
    if(tempoints.first == 3)
    {
      // cout<<"阶梯部分"<<endl;
      // int min_visit = 1;
      double nowvisit;
      double min_cost = 10000;
      double cost;
      int8_t now_esdf;

      Vector2i nowpoint;
      Vector2i tempoint;
      double last_distance = calPointLength(last,tempoints.second[0]);
      double end_distance = calPointLength(grid_map_end,tempoints.second[0]);
      // 参考代价数值
      double refer_esdf = getESDFvalue(tempoints.second[0]);
      double dangerous = refer_esdf/100.0f;   //越大越危险
      // 遍历搜索势场最小点数值
      for(int t=0;t<tempoints.second.size();t++)
      {
        // 获取势场数值
        nowpoint = tempoints.second[t];
        nowvisit = getVisitedNum(nowpoint);
        now_esdf = getESDFvalue(nowpoint);
        double distance = calPointLength(nowpoint,last);
        double dis_end = calPointLength(nowpoint,grid_map_end);
        // 定义系数，距离终点约近，则削弱势场所占比例
        double esdf_edis = dis_end/max_distance;
        esdf_edis = LIMIT_VALUE(esdf_edis,0.5,1.0);
        // 计算总的代价值，势场代价，到上一个点的距离代价，到终点的距离代价
        cost = (esdf_edis)*esdf_edis*Kstep_esdf*now_esdf/(refer_esdf+1.0) + (1-dangerous)*Kstep_ldis*distance/(last_distance+1.0)
        + (1-dangerous)*Kstep_edis*dis_end/(end_distance+1.0);
        if(cost < min_cost)
        {
          min_cost = cost;
          tempoint = nowpoint;
        }
      }
      last = tempoint;
      // 测试节点添加进来用于排序
      sampleSortPoints.push_back(make_pair(tempoints.second[0],tempoint));
    }
  }

  // 将起点和终点添加进来
  sampleSortPoints.insert(sampleSortPoints.begin(),make_pair(grid_map_start,grid_map_start));
  sampleSortPoints.push_back(make_pair(grid_map_end,grid_map_end));

  vector<pair<int,Vector2i>> testpoints;
  // 和原始路径进行对照遍历，按照原始路径点顺序排序
  for (int i = 0; i < sampleSortPoints.size(); i++)
  {
    // 和原始路径对照进行排序
    for (int j = 0; j < origingridPath.size(); j++)
    {
      // 按照原始路径点顺序排序
      if(origingridPath[j] == sampleSortPoints[j].first)
      {
        testpoints.push_back(make_pair(j,sampleSortPoints[i].second));
      }
    }
  }

  
  // 按照testpoints中第一项的顺序排序
  for (int i = 0; i < testpoints.size(); i++)
  {
    for (int j = 0; j < testpoints.size(); j++)
    {
      if(testpoints[j].first == i)
      {
        samplePoints.push_back(make_pair(0,testpoints[j].second));
        // optpath.push_back(testpoints[j].second);
        visualOptpoints.push_back(mapToWorld(testpoints[j].second));
      } 
    }
  }

  // 把起点放进来
  optpath.push_back(grid_map_start);

  pair<int,Vector2i> nowpoint;
  pair<int,Vector2i> lastpoint;
  // 开始搜索
  for (int i = 0; i < samplePoints.size()-1; i++)
  {
    double distance;
    double dis_end1;
    double dis_end2;
    int flag;
    double min_distance = 100000000;
    if( i == 0)
    {
      // 把起点放进来
      nowpoint = samplePoints[0];
      // 第一个点访问过，不重复访问
      samplePoints[0].first = 1;
    }
    pair<int,Vector2i> tempoint;
    // 进行判断
    // 设定一个向前搜索的步长度，如果这个长度内有点就搜，没有就挨个搜
    for (int j = 0; j < samplePoints.size(); j++)
    {
      // 计算距离
      distance = calPointLength(nowpoint.second,samplePoints[j].second);

      // 计算到终点的距离
      dis_end1 = calPointLength(samplePoints[j].second,grid_map_end);
      // 
      dis_end2 = calPointLength(nowpoint.second,grid_map_end);

      // 障碍物判定，到上一个点之间没有障碍物
      bool has_obstacle = CheckObstaclePoints(samplePoints[j].second,lastpoint.second);;
      
      // 当距离小于的时候，且没有被访问过的点
      // 用来消除不好的点
      if(distance < min_distance && samplePoints[j].first == 0 && j>i && has_obstacle == false && dis_end1<dis_end2)
      {
        // 传参
        min_distance = distance;
        // 获取当前点
        tempoint = samplePoints[j];
        // 合格的点都设定为1
        samplePoints[j].first = 1;
        // 获取j
        flag = j;
      }
      if(distance < min_distance && samplePoints[j].first == 0 && j>i)
      {
        // 传参
        min_distance = distance;
        // 获取当前点
        tempoint = samplePoints[j];
        // 合格的点都设定为1
        samplePoints[j].first = 1;
        // 获取j
        flag = j;
      }
    }

    // 判断flag 的大小
    // 好马不吃回头草
    if(flag==samplePoints.size()-1)
    {
      for (int num = 0; num < flag; num++)
      {
        /* code */
        samplePoints[num].first = 1;
      }
    }

    // 获取最近的节点
    nowpoint = tempoint;
    // 访问过则不重复访问
    samplePoints[flag].first = 1;

    optpath.push_back(nowpoint.second);
    lastpoint = nowpoint;
  }

  // 把终点添加进来
  optpath.push_back(grid_map_end);

  interpolation_path.clear();
  // 对路径进行插值
  Setlength_Interpolation_Path(0.1,optpath,interpolation_path);

  // 需要分段路径清空
  needivipath.clear();
  // 按照0.1的间距进行分段
  Setlength_DividPath(0.1,interpolation_path,needivipath);

  // 用来保存共线的断点在opt中的位置
  vector<int> collinear_Num = CollinearPosition(needivipath); 

  needoptpath.clear();
  // 将路径按照共线分段点进行分段
  DividPath(needivipath,collinear_Num,needoptpath);

  // 需要优化的路径清空
  notoptpath.clear();
  notoptpath = GridPathToWorldPath(optpath);
  
  worldpath.clear();
  // 使用贝塞尔进行优化
  worldpath = BezierPathOpt(needoptpath);
  // 把起点放进来
  worldpath.insert(worldpath.begin(),mapToWorld(grid_map_start));

  return worldpath;
}

// 获取走廊访问的节点
int Fast_Security::getCorridorVisitedNum(void)
{
  int visited_num = 0;
  // 遍历search_data 查看访问的节点数目
  for (int i = 0; i < search_data.size(); i++)
  {
    if(search_data[i] >= 1)
    {
      visited_num++;
    }
  }
  return visited_num;
}

// 获取最终优化过的路径的长度
double Fast_Security::getOptedPathLength(void)
{
  double length = 0.0;
  for (int i = 1; i < optedpath.size(); i++)
  {
    // 计算最终的距离
    length += calPointLength(optedpath[i],optedpath[i-1]);
  }
  
  return length;
}

void Fast_Security::visual_SamplesNode(ros::Publisher pathPublish, vector<vector<Vector2d>> samplenodes,
float a_set,float r_set,float g_set,float b_set,float k_length)
{
  visualization_msgs::Marker node_vis;
  node_vis.header.frame_id = "map";
  node_vis.header.stamp = ros::Time::now();

  node_vis.ns = "fast_security_visited";

  node_vis.type = visualization_msgs::Marker::CUBE_LIST;
  node_vis.action = visualization_msgs::Marker::ADD;
  
  node_vis.id = 0;
  node_vis.color.a = a_set;
  node_vis.color.r = r_set;
  node_vis.color.g = g_set;
  node_vis.color.b = b_set;

  node_vis.pose.orientation.x = 0.0;
  node_vis.pose.orientation.y = 0.0;
  node_vis.pose.orientation.z = 0.0;
  node_vis.pose.orientation.w = 1.0;

  node_vis.scale.x = resolution * k_length;
  node_vis.scale.y = resolution * k_length;
  node_vis.scale.z = resolution * k_length;

  geometry_msgs::Point pt;
  int sampleNum = samplenodes.size();
  for (int i = 0; i < sampleNum; i++)
  {
    vector<Vector2d> tempoints = samplenodes[i];
    int tempsize = tempoints.size(); 
    // 获取详细的点集合
    for (int j = 0; j < tempsize; j++)
    {
      pt.x = tempoints[j][0];
      pt.y = tempoints[j][1];
      node_vis.points.push_back(pt);
    }
    // cout<<"每个路径点扩展点数目:"<<tempsize<<endl;
  }

  pathPublish.publish(node_vis);  
}

void Fast_Security::visual_VisitedNode(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> visitnodes,
float a_set,float r_set,float g_set,float b_set,float length)
{
  visualization_msgs::Marker node_vis;
  node_vis.header.frame_id = "map";
  node_vis.header.stamp = ros::Time::now();

  node_vis.color.a = a_set;
  node_vis.color.r = r_set;
  node_vis.color.g = g_set;
  node_vis.color.b = b_set;
  node_vis.ns = "fast_security_visited";

  node_vis.type = visualization_msgs::Marker::CUBE_LIST;
  node_vis.action = visualization_msgs::Marker::ADD;
  node_vis.id = 0;

  node_vis.pose.orientation.x = 0.0;
  node_vis.pose.orientation.y = 0.0;
  node_vis.pose.orientation.z = 0.0;
  node_vis.pose.orientation.w = 1.0;

  node_vis.scale.x = resolution*length;
  node_vis.scale.y = resolution*length;
  node_vis.scale.z = resolution*length;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(visitnodes.size()); i++)
  {
    pt.x = visitnodes[i][0];
    pt.y = visitnodes[i][1];
    node_vis.points.push_back(pt);
  }

  pathPublish.publish(node_vis);
}







