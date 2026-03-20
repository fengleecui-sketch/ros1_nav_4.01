#include "path_searcher/two_way_rrt.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

void RRTstarPlanner::initMap(double _resolution,double _originx,double _originy,
        unsigned int _x_size,unsigned int _y_size,std::vector<int> _mapData)
{
    //获取分辨率
    resolution_ = _resolution;
    //获取地图防止在世界坐标系下的坐标
    origin_x = _originx;    
    origin_y = _originy;
    // 获取地图尺寸
    X_SIZE = _x_size;
    Y_SIZE = _y_size;

    // 初始化一个数组,按照XYZ的大小去初始化数组
    costmap2d = new uint8_t[Y_SIZE*X_SIZE];     //为将地图转化为8进制栅格地图作准备
    // 内存处理,清空数组
    memset(costmap2d, 0, Y_SIZE*X_SIZE * sizeof(uint8_t));

    for(unsigned int i=0;i<X_SIZE;i++)
    {
        for(unsigned int j=0;j<Y_SIZE;j++)
        {
            // 设定障碍物
            if(_mapData[i*Y_SIZE+j] == INSCRIBED_INFLATED_OBSTACLE)
            {
                costmap2d[i*Y_SIZE+j] = INSCRIBED_INFLATED_OBSTACLE; // data用来储存对应栅格点有没有障碍物
            }
        }
    }

    search_radius_ = 1.0;
    goal_radius_ = 0.2;
    epsilon_min_ = 0.001;
    epsilon_max_ = 0.1;
    max_nodes_num_ = 2000000000;

    // 路径优化参数
    path_point_spacing_ = 0.025; //路径点间隔
    angle_difference_ = M_PI/20; //前后相邻点向量角度差

    std::cout<<"地图初始化完毕:"<<std::endl;
}












void RRTstarPlanner::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy)const
{
    wx = origin_x + (mx+0.5)*resolution_;
    wy = origin_y + (my+0.5)*resolution_;
}

bool RRTstarPlanner::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  if (wx < origin_x || wy < origin_y)
    return false;

  mx = (int)((wx - origin_x) / resolution_);
  my = (int)((wy - origin_y) / resolution_);

  if (mx < X_SIZE && my < Y_SIZE)
    return true;

  return false;
}

unsigned char RRTstarPlanner::getCost(unsigned int mx, unsigned int my) const
{
  return costmap2d[getIndex(mx, my)];
}

// 获取系统时间最小单位是ns，在ros中返回值为获取到的ros的时间
uint64_t RRTstarPlanner::getSystemNSec(void)
{
    return ros::Time::now().toNSec();
}

//得到路径，两棵树连接在了一起
void RRTstarPlanner::getPathFromTree(std::vector< Node >& tree1,
                                        std::vector< Node >& tree2,
                                        Node& connect_node,
                                        std::vector<Eigen::Vector2d>& plan,
                                        GetPlanMode mode)
{
    std::pair<double, double> point;
    std::vector< std::pair<double, double> > path;
    Node current_node;

    //第一棵树搜索到第二课树上的节点
    if(mode == GetPlanMode::CONNECT1TO2 )
    {
        current_node = tree1.back();
    }

    //第一棵树搜索到路径点
    if(GetPlanMode::TREE1)
    {
        current_node = tree1.back();
    }

    // 第二棵树搜索到第一棵上的节点
    if(mode == GetPlanMode::CONNECT2TO1)
    {
        current_node = connect_node;
    }

    //第二棵树搜索到路径
    if(mode == GetPlanMode::TREE2)
    {
        current_node = tree1[0];
    }

    // Final Path
    while (current_node.parent_id != tree1[0].parent_id)
    {
        point.first = current_node.x;
        point.second = current_node.y;
        path.insert(path.begin(), point); //在开头插入一个元素
        current_node = tree1[current_node.parent_id];
    }

    if(mode == GetPlanMode::CONNECT1TO2) //1->2
    {
        current_node = connect_node;
    }

    if(mode == GetPlanMode::TREE1) //1->goal
    {
        current_node = tree2[0];
    }

    if(mode == GetPlanMode::TREE2) //2->start
    {
        current_node = tree2.back();
    }

    if(mode == GetPlanMode::CONNECT2TO1) //2->1
    {
        current_node = tree2.back();
    }

    while (current_node.parent_id != tree2[0].parent_id)
    {
        point.first = current_node.x;
        point.second = current_node.y;
        path.push_back(point);
        current_node = tree2[current_node.parent_id];
    }

    point.first = tree1[0].x;
    point.second = tree1[0].y;
    path.insert(path.begin(), point); //开始点
    point.first = tree2[0].x;
    point.second = tree2[0].y;
    path.push_back(point); //目标点

    cutPathPoint(path);     //裁剪路径上的节点

    insertPointForPath(path,this->path_point_spacing_); //插入节点

    optimizationPath(path,this->angle_difference_);     //优化路径,因为规划出来的路径是不连续的,采用优化的方法让前后两个点的角度差保持在一个范围内

    for (size_t i = 0; i < path.size(); i++)
    {
        Eigen::Vector2d pose;
        pose[0] = path[i].first;
        pose[1] = path[i].second;
        plan.push_back(pose);
    }
}

// 发布路径
// param1 要发布的路径
void RRTstarPlanner::PublishPath(std::vector<Eigen::Vector2d> path)
{
    path[0][0] = startnode[0];
    path[0][1] = startnode[1];

    path[path.size()-1][0] = targetnode[0];
    path[path.size()-1][1] = targetnode[1];

    nav_msgs::Path path_pose;
    path_pose.poses.clear();

    for(unsigned int i=0;i<path.size();i++)
    {
        geometry_msgs::PoseStamped pathPose;
        pathPose.pose.position.x = path[i][0];
        pathPose.pose.position.y = path[i][1];
        pathPose.pose.position.z=0;
        pathPose.pose.orientation.x = 0.0;
        pathPose.pose.orientation.y = 0.0;
        pathPose.pose.orientation.z = 0.0;
        pathPose.pose.orientation.w = 1.0;

        path_pose.header.frame_id = "odom";
        path_pose.header.stamp = ros::Time::now();
        path_pose.poses.push_back(pathPose);
    }

    plan_pub_.publish(path_pose);   //发布路径
}











//标准化角度  
// 角度数值
// 角度最大范围值
// 角度最小范围值
double RRTstarPlanner::normalizeAngle(double val,double min,double max) //标准化角度
{
    double norm = 0.0;
    if (val >= min)
        norm = min + fmod((val - min), (max-min));
    else
        norm = max - fmod((min - val), (max-min));

    return norm;
}

//发布整个树的可视化扩展
void RRTstarPlanner::pubTreeMarker(ros::Publisher &marker_pub, visualization_msgs::Marker marker,int id)
{
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_namespace";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
    marker.frame_locked = false;

    marker_pub.publish(marker);
}

bool RRTstarPlanner::FindPath(Eigen::Vector2d start,Eigen::Vector2d target)
{
    startnode = start;
    targetnode = target;

    if(this->collision(start[0],start[1]))
    {
        std::cout<<"start point is"<<start[0]<<"  "<<start[1]<<std::endl;
        std::cout<<"failed to get a path.start point is obstacle."<<std::endl;
        return false;
    }
    if(this->collision(target[0],target[1]))
    {
        std::cout<<"target point is"<<target[0]<<"  "<<target[1]<<std::endl;
        std::cout<<"failed to get a path.target point is obstacle."<<std::endl;    
        return false;    
    }

    this->tree1.clear();    //对第一棵树进行清0
    this->tree2.clear();    //对第二棵树进行清0

    path.clear();       //路径清空

    std::vector< Node > nodes_1; //第一棵树
    Node start_;
    start_.x = start[0];
    start_.y = start[1];
    start_.node_id = 0;
    start_.parent_id = -1;
    start_.cost = 0.0;
    nodes_1.push_back(start_);

    std::vector< Node > nodes_2; //第二棵树
    Node target_;
    target_.x = target[0];
    target_.y = target[1];
    target_.node_id = 0;
    target_.parent_id = -1;
    target_.cost = 0.0;
    nodes_2.push_back(target_);

    std::pair<double, double> p_rand; //随机采样的可行点
    std::pair<double, double> p_new; //第一棵树的新节点
    std::pair<double, double> p_new_2; //第二棵树的新节点
    Node connect_node_on_tree1; //第二课树与第一课树连接到一起时第一课树上距离第二课树最近的节点
    Node connect_node_on_tree2; //第一课树与第二课树连接到一起时第二课树上距离第二课树最近的节点

    bool is_connect_to_tree1 = false;
    bool is_connect_to_tree2 = false;

    Node node_nearest;
    unsigned int seed = 0;
    while (1 && nodes_1.size() + nodes_2.size() < max_nodes_num_)
    {
        // 第一棵树  从起点向终点搜索
        while (1)
        {
            srand(getSystemNSec() + seed++);//修改种子
            unsigned int rand_nu = rand()%10;

            if(rand_nu > 1) // 0.8的概率使用随机采样扩展
            {
                p_rand = sampleFree(); // random point in the free space
            }
            else // 0.2的概率使用启发扩展
            {
                p_rand.first = target[0];
                p_rand.second = target[1];
            }
                    
            node_nearest = getNearest(nodes_1, p_rand); // 获取随即节点中的最近节点
            p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second); // new point and node candidate.
            if (obstacleFree(node_nearest, p_new.first, p_new.second))
            {//树枝无碰撞
                Node newnode;
                newnode.x = p_new.first;
                newnode.y = p_new.second;
                newnode.node_id = nodes_1.size(); // index of the last element after the push_bask below
                newnode.parent_id = node_nearest.node_id;       //父节点id为最近的子节点id,进行循环搜索
                newnode.cost = 0.0;

                // 优化
                newnode = chooseParent(node_nearest, newnode, nodes_1); // Select the best parent
                nodes_1.push_back(newnode);
                rewire(nodes_1, newnode);
                
                // 将探索的枝叶进行添加，获取到第一棵树
                Eigen::Vector2d pointTem;
                pointTem[0] = nodes_1[newnode.parent_id].x;
                pointTem[1] = nodes_1[newnode.parent_id].y;
                this->tree1.push_back(pointTem);

                pointTem[0] = newnode.x;
                pointTem[1] = newnode.y;
                this->tree1.push_back(pointTem);

                // if(nodes.size() % 10 == 0)
                // {
                //     this->pubTreeMarker(this->marker_pub_,this->marker_tree_,1);
                // }

                if(this->isConnect(newnode,nodes_2,nodes_1, connect_node_on_tree2))   //判断和第二棵树是否连接上
                {
                    is_connect_to_tree2 = true;
                }

                break;
            }
        }
        //两棵树连接在了一起,第一棵树搜索到了第二棵树上的节点
        if(is_connect_to_tree2)
        {
            getPathFromTree(nodes_1,nodes_2,connect_node_on_tree2,path,GetPlanMode::CONNECT1TO2); //从搜索树中获取路径
            return true;
        }

        // 第一棵树搜索到目标点
        if (pointCircleCollision(p_new.first, p_new.second,target[0],target[1], goal_radius_) )
        {
            getPathFromTree(nodes_1,nodes_2,nodes_1.back(),path,GetPlanMode::TREE1);
            return true;
        }

        // 第二棵树  从终点向起点搜索
        p_rand.first = p_new.first;
        p_rand.second = p_new.second;
        while (1)
        {
            node_nearest = getNearest(nodes_2, p_rand); // The nearest node of the random point
            p_new_2 = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second); // new point and node candidate.
            if (obstacleFree(node_nearest, p_new_2.first, p_new_2.second))
            {
                Node newnode;
                newnode.x = p_new_2.first;
                newnode.y = p_new_2.second;
                newnode.node_id = nodes_2.size(); // index of the last element after the push_bask below
                newnode.parent_id = node_nearest.node_id;
                newnode.cost = 0.0;

                // Optimize
                newnode = chooseParent(node_nearest, newnode, nodes_2); // Select the best parent
                nodes_2.push_back(newnode);
                rewire(nodes_2, newnode);

                // 将探索的枝叶进行添加，获取到第二棵树
                Eigen::Vector2d pointTem;
                pointTem[0] = nodes_1[newnode.parent_id].x;
                pointTem[1] = nodes_1[newnode.parent_id].y;
                this->tree2.push_back(pointTem);

                pointTem[0] = newnode.x;
                pointTem[1] = newnode.y;
                this->tree2.push_back(pointTem);

                // if(nodes_2.size() % 10 == 0)
                // {
                //     pubTreeMarker(this->marker_pub_,this->marker_tree_2_,2);
                // }

                if(this->isConnect(newnode,nodes_1,nodes_2, connect_node_on_tree1))   //判断和第一棵树是否连接上
                {
                    is_connect_to_tree1 = true;
                }

                break;
            }
            else
            {
                srand(getSystemNSec() + seed++);//修改种子
                unsigned int rand_nu = rand()%10;
                if(rand_nu > 1) // 0.8的概率使用随机采样扩展
                {
                    p_rand = sampleFree(); // random point in the free space 在可通过区域进行采样
                }
                else // 0.2的概率使用启发扩展
                {
                    p_rand.first = start[0];
                    p_rand.second = start[1];
                }
            }
        }

        //两棵树连接在了一起，第二棵树搜索到了第一棵树上的节点
        if(is_connect_to_tree1)
        {
            std::cout << "两棵树连接在了一起"<< std::endl;

            getPathFromTree(nodes_1,nodes_2,connect_node_on_tree1,path,GetPlanMode::CONNECT2TO1);
            return true;
        }

        //第二棵树搜索到目标点  中间没有遇见,重新由终点搜索到了起点
        if (pointCircleCollision(p_new_2.first, p_new_2.second,start[0],start[1], goal_radius_) )
        {
            std::cout << "第二棵树搜索到目标点"<< std::endl;

            getPathFromTree(nodes_1,nodes_2,nodes_1.front(),path,GetPlanMode::TREE2);
            return true;
        }
    }   
    ROS_WARN("failed to get a path.");
    return false;
}

//计算新节点与目标点的距离是否小于位置允许误差
bool RRTstarPlanner::pointCircleCollision(double x1, double y1, double x2, double y2, double goal_radius)
{
    double dist = distance(x1, y1, x2, y2);
    if (dist < goal_radius)
        return true;
    else
        return false;
}

//路径插点      将最终的路径连成一条线
void RRTstarPlanner::insertPointForPath(std::vector<std::pair<double, double> >& path_in,double param)
{
    std::vector<std::pair<double, double> > path_out;
    size_t size = path_in.size() - 1;
    std::pair<double, double> point;
    double pp_dist = param;
    for(size_t i=0;i<size;i++)
    {
        double theta = atan2(path_in[i+1].second - path_in[i].second,path_in[i+1].first - path_in[i].first);
        size_t insert_size = static_cast<size_t>(this->distance(path_in[i+1].first,path_in[i+1].second,path_in[i].first,path_in[i].second) / pp_dist + 0.5);
        for(size_t j=0;j<insert_size;j++)
        {
            point.first = path_in[i].first + j * pp_dist * cos(theta);
            point.second = path_in[i].second + j * pp_dist * sin(theta);
            path_out.push_back(point);
        }
    }
    path_out.push_back( path_in.back() );
    path_in.clear();
    size = path_out.size();
    path_in.resize(size);
    for(size_t i=0;i<size;i++)
    {
        path_in[i] = path_out[i];
    }
}

// 优化路径
int RRTstarPlanner::optimizationPath(std::vector<std::pair<double, double> >& plan,double movement_angle_range)
{
    if(plan.empty())
        return 0;
    size_t point_size = plan.size() - 1;
    double px,py,cx,cy,nx,ny,a_p,a_n;
    bool is_run = false;
    int ci = 0;
    for(ci=0;ci<1000;ci++)
    {
        is_run = false;
        for(size_t i=1;i<point_size;i++)
        {
            px = plan[i-1].first;
            py = plan[i-1].second;

            cx = plan[i].first;
            cy = plan[i].second;

            nx = plan[i+1].first;
            ny = plan[i+1].second;

            a_p = normalizeAngle(atan2(cy-py,cx-px),0,2*M_PI);
            a_n = normalizeAngle(atan2(ny-cy,nx-cx),0,2*M_PI);

            if(std::max(a_p,a_n)-std::min(a_p,a_n) > movement_angle_range)
            {
                plan[i].first = (px + nx)/2;
                plan[i].second = (py + ny)/2;
                is_run = true;
            }
        }
        if(!is_run)
        return ci;
    }
    return ci;
}

// 判断两个点之间有没有障碍物
bool RRTstarPlanner::isLineFree(const std::pair<double, double> p1, const std::pair<double, double> p2)
{
    std::pair<double, double> ptmp;
    ptmp.first = 0.0;
    ptmp.second = 0.0;

    double dist = sqrt( (p2.second-p1.second) * (p2.second-p1.second) +
                        (p2.first-p1.first) * (p2.first-p1.first) );
    if (dist < this->resolution_)
    {
        return true;
    }
    else
    {
        int value = int(floor(dist/this->resolution_));
        double theta = atan2(p2.second - p1.second,
                            p2.first - p1.first);
        int n = 1;
        for (int i = 0;i < value; i++)
        {
        ptmp.first = p1.first + this->resolution_*cos(theta) * n;
        ptmp.second = p1.second + this->resolution_*sin(theta) * n;
        if (collision(ptmp.first, ptmp.second))
            return false;
        n++;
        }
        return true;
    }
}

// 裁剪路径上的节点
void RRTstarPlanner::cutPathPoint(std::vector<std::pair<double, double> >& plan)
{
    size_t current_index = 0;
    size_t check_index = current_index+2;
    while(ros::ok())
    {
        if( current_index >= plan.size()-2 )
            return;
        if( this->isLineFree(plan[current_index], plan[check_index]) ) //点之间无障碍物
        {
            std::vector<std::pair<double, double> >::iterator it = plan.begin()+ static_cast<int>(current_index + 1) ;
            if(check_index-current_index-1 == 1)
            {
                plan.erase(it);
            }
            else
            {
                plan.erase(it,it+static_cast<int>( check_index-current_index-1) );
                check_index = current_index + 2;
            }
        }
        else
        {
            if(check_index < plan.size()-1 )
                check_index++;
            else
            {
                current_index++;
                check_index = current_index + 2;
            }
        }
    }
}

//计算两点间距离  欧氏距离
double RRTstarPlanner::distance(double px1, double py1, double px2, double py2)
{
    return sqrt((px1 - px2)*(px1 - px2) + (py1 - py2)*(py1 - py2));
}

//随机采样
std::pair<double, double> RRTstarPlanner::sampleFree()
{
    std::pair<double, double> random_point;
    unsigned int mx = 0,my = 0;
    double wx = 0.0,wy = 0.0;
    unsigned int map_size_x = X_SIZE;
    unsigned int map_size_y = Y_SIZE;
    unsigned int seed = 0;
    while(1)
    {
        srand(getSystemNSec() + seed++);//修改种子
        mx = rand() % map_size_x;
        srand(getSystemNSec() + seed++);//修改种子
        my = rand() % map_size_y;
        if(getCost(mx,my) < INSCRIBED_INFLATED_OBSTACLE)
        break;
    }
    mapToWorld(mx,my,wx,wy);
    random_point.first = wx;
    random_point.second = wy;
    return random_point;
}

//检查点是否为障碍物
bool RRTstarPlanner::collision(double x, double y)
{
    unsigned int mx,my;
    if(!worldToMap(x,y,mx,my))
        return true;
    if ((mx >= X_SIZE) || (my >= Y_SIZE))
        return true;
    if (getCost(mx, my) >= INSCRIBED_INFLATED_OBSTACLE)
        return true;

    return false;
}

//检查点的周围是否有障碍物
bool RRTstarPlanner::isAroundFree(double wx, double wy)
{
    unsigned int mx, my;
    if(!worldToMap(wx,wy,mx,my))
        return false;
    if(mx <= 1 || my <= 1 || mx >= X_SIZE-1 || my >= Y_SIZE-1)
        return false;
    int x,y;
    for(int i=-1;i<=1;i++)
    {
        for(int j=-1;j<=1;j++)
        {
            x = static_cast<int>(mx) + i;
            y = static_cast<int>(my) + j;
            if(getCost(static_cast<unsigned int>(x),static_cast<unsigned int>(y)) >= INSCRIBED_INFLATED_OBSTACLE)
                return false;
        }
    }
    return true;
}

bool RRTstarPlanner::isConnect(Node new_node,std::vector< Node >& another_tree,std::vector< Node >& current_tree,Node& connect_node)
{
    size_t node_size = another_tree.size();
    double min_distance = 10000000.0;
    double distance = 0.0;
    size_t min_distance_index = 0;
    for(size_t i=0;i<node_size;i++)
    {
        distance = this->distance(new_node.x,new_node.y,another_tree[i].x,another_tree[i].y);
        if(distance < min_distance)
        {
            min_distance = distance;
            min_distance_index = i;
        }
    }

    distance = this->distance(new_node.x,new_node.y,another_tree[min_distance_index].x,another_tree[min_distance_index].y);

    if(distance < this->goal_radius_)
    {
        connect_node = another_tree[min_distance_index];
        Node newnode = another_tree[min_distance_index];
        // Optimize
        newnode = chooseParent(current_tree.back(), newnode, current_tree); // Select the best parent
        current_tree.push_back(newnode);
        rewire(current_tree, newnode);
        return true;
    }
    return false;
}

//搜索树上距离最近的节点
Node RRTstarPlanner::getNearest(std::vector< Node > nodes, std::pair<double, double> p_rand)
{

    double dist_min = distance(nodes[0].x, nodes[0].y, p_rand.first, p_rand.second);
    double dist_curr = 0;
    size_t index_min = 0;
    for (size_t i = 1; i < nodes.size(); i++)
    {
        dist_curr = distance(nodes[i].x, nodes[i].y, p_rand.first, p_rand.second);
        if (dist_curr < dist_min)
        {
            dist_min = dist_curr;
            index_min = i;
        }
    }

    return nodes[index_min];
}

//选择最优的父节点
Node RRTstarPlanner::chooseParent(Node nn, Node newnode, std::vector<Node> nodes)
{
    double dist_nnn = distance(nn.x, nn.y, newnode.x, newnode.y);
    for (size_t i = 0; i < nodes.size(); i++)
    {
        if (distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < search_radius_ &&
            nodes[i].cost + distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < nn.cost + dist_nnn &&
            obstacleFree(nodes[i], newnode.x, newnode.y))
        {
        nn = nodes[i];
        }
    }
    newnode.cost = nn.cost + distance(nn.x, nn.y, newnode.x, newnode.y);
    if(!this->isAroundFree(newnode.x,newnode.y))
        newnode.cost += 0.3;
    newnode.parent_id = nn.node_id;
    return newnode;
}

//优化新节点附近的节点，选择最优的父节点
void RRTstarPlanner::rewire(std::vector<Node>& nodes, Node newnode)
{
    for (size_t i = 0; i < nodes.size(); i++)
    {
        Node& node = nodes[i];
        if (node != nodes[newnode.parent_id] &&
            distance(node.x, node.y, newnode.x, newnode.y) < search_radius_ &&
            newnode.cost + distance(node.x, node.y, newnode.x, newnode.y) < node.cost &&
            obstacleFree(node, newnode.x, newnode.y))
        {
            node.parent_id = newnode.node_id;
            node.cost = newnode.cost + distance(node.x, node.y, newnode.x, newnode.y);
            if(!this->isAroundFree(node.x,node.y))
                node.cost += 0.3;
        }
    }
}

//在树上扩展新节点
std::pair<double, double> RRTstarPlanner::steer(double x1, double y1,double x2, double y2)
{
    std::pair<double, double> p_new;
    double dist = distance(x1, y1, x2, y2);
    if (dist < epsilon_max_ && dist > epsilon_min_)
    {
        p_new.first = x1;
        p_new.second = y1;
    }
    else
    {
        double theta = atan2(y2-y1, x2-x1);
        p_new.first = x1 + epsilon_max_*cos(theta);
        p_new.second = y1 + epsilon_max_*sin(theta);
    }
    return p_new;
}

//判断两节点之间是否有障碍物
bool RRTstarPlanner::obstacleFree(Node node_nearest, double px, double py)
{
    std::pair<double, double> p_n;
    p_n.first = 0.0;
    p_n.second = 0.0;

    double dist = distance(node_nearest.x, node_nearest.y, px, py);
    if (dist < resolution_) //当距离障碍物的距离小于分辨率  判断为障碍物
    {
        if (collision(px, py))
            return false;
        else
            return true;
    }
    else
    {
        int value = int(floor(dist/resolution_));
        double theta = atan2(py - node_nearest.y, px - node_nearest.x);
        int n = 1;
        for (int i = 0;i < value; i++)
        {
            p_n.first = node_nearest.x + n*resolution_*cos(theta);
            p_n.second = node_nearest.y + n*resolution_*sin(theta);
            if (collision(p_n.first, p_n.second))
                return false;
            n++;
        }
        return true;
    }
}

