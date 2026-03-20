#ifndef __NODE_H
#define __NODE_H

#include "iostream"
#include "ros/ros.h"
#include "ros/console.h"
#include "Eigen/Eigen"

#define inf 1>>20  //1/(2^20)
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{
    int id;     //1-->open set,-1-->closed set
    Eigen::Vector2i dir;    //direction of enpanding
    Eigen::Vector2i index;

    double gScore,fScore;   //分类模型评估方法f分数的两个参数
    double aclDistance;        //定义一个用于计算A*实际走过的距离的参数 
    GridNodePtr cameFrom;

    std::multimap<double,GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector2i _index)
    {
        id=0;
        index=_index;
        dir =Eigen::Vector2i::Zero();

        gScore=inf;         //相当于无穷小的一个数,可看作归0
        fScore=inf;         //同上
        aclDistance = inf;
        cameFrom=NULL;
    }

    GridNode(){};
    ~GridNode(){};
};


#endif

