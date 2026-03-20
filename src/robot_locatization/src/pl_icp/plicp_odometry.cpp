/*
 * Copyright 2020 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "pl_icp/plicp_odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

ScanMatchPLICP::ScanMatchPLICP() : private_node_("~"), tf_listener_(tfBuffer_)
{
	// \033[1;32m，\033[0m 终端显示成绿色
	ROS_INFO_STREAM("\033[1;32m----> PLICP odometry started.\033[0m");

	// 获取激光雷达的类型，仿真的雷达还是真正的雷达
	private_node_.param<std::string>("laserType",laserType,"laser_scan");
	// 雷达消息订阅
	laser_scan_subscriber_ = node_handle_.subscribe(
			laserType, 1, &ScanMatchPLICP::ScanCallback, this);
	
	// 获取里程计的类型，仿真的里程计还是实际的里程计，这里默认取的是仿真的里程计
	private_node_.param<std::string>("odomType",odomType,"odom");
	// 里程计消息订阅
	odom_subscriber = node_handle_.subscribe(odomType, 1, &ScanMatchPLICP::OdomCallback, this);

	odom_publisher_ = node_handle_.advertise<nav_msgs::Odometry>("odom_plicp", 50);

	// 参数初始化
	InitParams();

	scan_count_ = 0;

	// 第一帧雷达还未到来
	initialized_ = false;

	base_in_odom_.setIdentity();
	base_in_odom_keyframe_.setIdentity();

	input_.laser[0] = 0.0;
	input_.laser[1] = 0.0;
	input_.laser[2] = 0.0;

	// Initialize output_ vectors as Null for error-checking
	output_.cov_x_m = 0;
	output_.dx_dy1_m = 0;
	output_.dx_dy2_m = 0;
}

ScanMatchPLICP::~ScanMatchPLICP()
{
}

/*
 * ros与csm的参数初始化
 */
void ScanMatchPLICP::InitParams()
{
	private_node_.param<std::string>("odom_frame", odom_frame_, "odom");
	private_node_.param<std::string>("base_frame", base_frame_, "base_footprint");
	//***关键帧参数：何时生成关键帧扫描
	// 如果任意一个设置为0，则减少到帧对帧匹配
	private_node_.param<double>("kf_dist_linear", kf_dist_linear_, 0.1);
	private_node_.param<double>("kf_dist_angular", kf_dist_angular_, 5.0 * (M_PI / 180.0));
	kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;
	private_node_.param<int>("kf_scan_count", kf_scan_count_, 10);

	// **** CSM 的参数 - comments copied from algos.h (by Andrea Censi)

	// getParam是获取必须的参数的，可以获取launch文件或者yaml文件中的参数
	// 扫描之间的最大角位移
	if (!private_node_.getParam("max_angular_correction_deg", input_.max_angular_correction_deg))
		input_.max_angular_correction_deg = 45.0;

	// 扫描之间最大位移 m
	if (!private_node_.getParam("max_linear_correction", input_.max_linear_correction))
		input_.max_linear_correction = 1.0;

	// 最大ICP周期迭代次数
	if (!private_node_.getParam("max_iterations", input_.max_iterations))
		input_.max_iterations = 10;

	// 停车阈值（m）
	if (!private_node_.getParam("epsilon_xy", input_.epsilon_xy))
		input_.epsilon_xy = 0.000001;

	// 停止阈值（弧度）
	if (!private_node_.getParam("epsilon_theta", input_.epsilon_theta))
		input_.epsilon_theta = 0.000001;

	// 点云匹配中两个点之间的最大距离  过小导致匹配的点数过少，过大匹配错误的点
	if (!private_node_.getParam("max_correspondence_dist", input_.max_correspondence_dist))
		input_.max_correspondence_dist = 1.0;

	// 扫描中的噪声（m）
	// sigma参数通常是在点云匹配的过程中用于计算距离权重的。可以根据具体的应用场景来选择合适的sigma值，以提高匹配的准确性和速度
	if (!private_node_.getParam("sigma", input_.sigma))
		input_.sigma = 0.010;

	// 即为了使用智能技巧来查找相应点对
	// use_corr_tricks是一个bool类型的参数，用于控制点云匹配过程中是否使用一些技巧来提高匹配的准确性和速度。
	if (!private_node_.getParam("use_corr_tricks", input_.use_corr_tricks))
		input_.use_corr_tricks = 1;

	// 重新启动：如果错误超过阈值，则重新启动
	if (!private_node_.getParam("restart", input_.restart))
		input_.restart = 0;

	// 重新启动：重新启动阈值
	if (!private_node_.getParam("restart_threshold_mean_error", input_.restart_threshold_mean_error))
		input_.restart_threshold_mean_error = 0.01;

	// 重新启动：用于重新启动时间，超过这个时间算法重启
	if (!private_node_.getParam("restart_dt", input_.restart_dt))
		input_.restart_dt = 1.0;

	// 重新启动：用于重新启动的置换。（弧度）
	if (!private_node_.getParam("restart_dtheta", input_.restart_dtheta))
		input_.restart_dtheta = 0.1;

	// 在同一集群中停留的最大距离
	if (!private_node_.getParam("clustering_threshold", input_.clustering_threshold))
		input_.clustering_threshold = 0.25;

	// 用于估计方向的相邻光线数
	if (!private_node_.getParam("orientation_neighbourhood", input_.orientation_neighbourhood))
		input_.orientation_neighbourhood = 20;

	// 如果为0，则为vanillaICP
	if (!private_node_.getParam("use_point_to_line_distance", input_.use_point_to_line_distance))
		input_.use_point_to_line_distance = 1;

	// 根据角度丢弃对应关系
	if (!private_node_.getParam("do_alpha_test", input_.do_alpha_test))
		input_.do_alpha_test = 0;

	// 根据角度丢弃对应关系-阈值角度，单位为度
	if (!private_node_.getParam("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
		input_.do_alpha_test_thresholdDeg = 20.0;

	// 需要考虑的信函百分比：如果为0.9，
	// 总是丢弃错误较多的前10%的对应关系
	if (!private_node_.getParam("outliers_maxPerc", input_.outliers_maxPerc))
		input_.outliers_maxPerc = 0.90;

	// 描述用于丢弃的简单自适应算法的参数。
	// 1）对错误进行排序。
	// 2）根据outliers_adaptive_order选择百分比。
	// （如果是0.7，则获得70%的百分比）
	// 3）定义自适应阈值乘以outliers_adaptive_ult
	// 其中误差的值在所选择的百分位处。
	// 4）丢弃超过阈值的对应关系。
	// 保守一点是有用的；但消除了最大的错误。
	if (!private_node_.getParam("outliers_adaptive_order", input_.outliers_adaptive_order))
		input_.outliers_adaptive_order = 0.7;

	if (!private_node_.getParam("outliers_adaptive_mult", input_.outliers_adaptive_mult))
		input_.outliers_adaptive_mult = 2.0;

	// 如果你已经对解决方案有了猜测，你可以计算极角
	// 在新位置进行一次扫描的点。如果极角不是单调的
	// 读数指数的函数，这意味着在
	// 下一个位置。如果它不可见，那么我们就不会使用它进行匹配。
	if (!private_node_.getParam("do_visibility_test", input_.do_visibility_test))
		input_.do_visibility_test = 0;

	// laser_sens中没有两个点可以具有相同的corr。
	if (!private_node_.getParam("outliers_remove_doubles", input_.outliers_remove_doubles))
		input_.outliers_remove_doubles = 1;

	// 如果为1，则使用以下方法计算ICP的协方差 http://purl.org/censi/2006/icpcov
	if (!private_node_.getParam("do_compute_covariance", input_.do_compute_covariance))
		input_.do_compute_covariance = 0;

	// 检查find_correspoondences_tricks是否给出正确答案
	if (!private_node_.getParam("debug_verify_tricks", input_.debug_verify_tricks))
		input_.debug_verify_tricks = 0;

	// 如果为1，则第一次扫描中的字段“true_alpha”（或“alpha”）用于计算
	// 发病率β和用于加权对应关系的因子（1/cos^2（β））。");
	if (!private_node_.getParam("use_ml_weights", input_.use_ml_weights))
		input_.use_ml_weights = 0;

	// 如果为1，则第二次扫描中的字段“readings_sigma”用于对
	// 1/sigma^2对应
	if (!private_node_.getParam("use_sigma_weights", input_.use_sigma_weights))
		input_.use_sigma_weights = 0;
}

// odom里程计回调函数
void ScanMatchPLICP::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_sensor_msg)
{
	// 麦轮车体速度合成
	linear_x = odom_sensor_msg->twist.twist.linear.x;
	linear_y = odom_sensor_msg->twist.twist.linear.y;

	angular = odom_sensor_msg->twist.twist.angular.z;

	// 四元数转换欧拉角 初始化并赋值
	tf2::Quaternion quat(odom_sensor_msg->pose.pose.orientation.x,
											 odom_sensor_msg->pose.pose.orientation.y,
											 odom_sensor_msg->pose.pose.orientation.z,
											 odom_sensor_msg->pose.pose.orientation.w);
	// 四元数向欧拉角进行转换 对local的角度
	tf2::Matrix3x3(quat).getRPY(local.roll, local.pitch, local.yaw);
	// 对local速度赋值
	local.Vx = linear_x;
	local.Vy = linear_y;

	// 换成麦轮模型之后的飘逸问题是因为获得的数据是车的局部速度数据，现l在将数据转换为全局速度数据
	vel_transform::LocalVelocityToGlobal(&local, &global);

	// if (abs(global.Vx) > 0.1 || abs(global.Vy) > 0.1)
	// {
	// 	ROS_INFO("global_vx %f,global_vy %f", global.Vx, global.Vy);
	// }

	latest_velocity_.linear.x = global.Vx;
	latest_velocity_.linear.y = global.Vy;
	latest_velocity_.angular.z = angular;
}

/*
 * 回调函数 进行数据处理
 */
void ScanMatchPLICP::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
	// 如果是第一帧数据，首先进行初始化，先缓存一下cos与sin值
	// 将 prev_ldp_scan_,last_icp_time_ 初始化
	current_time_ = scan_msg->header.stamp;
	if (!initialized_)
	{
		// 缓存所有角度的sin和cos
		CreateCache(scan_msg);

		// 获取机器人坐标系与雷达坐标系间的坐标变换
		if (!GetBaseToLaserTf(scan_msg->header.frame_id))
		{
			ROS_WARN("Skipping scan");
			return;
		}

		LaserScanToLDP(scan_msg, prev_ldp_scan_);
		last_icp_time_ = current_time_;
		initialized_ = true;
		return;
	}

	// step1 进行数据类型转换
	start_time_ = std::chrono::steady_clock::now();

	LDP curr_ldp_scan;
	LaserScanToLDP(scan_msg, curr_ldp_scan);

	end_time_ = std::chrono::steady_clock::now();
	time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
	// std::cout << "\n转换数据格式用时: " << time_used_.count() << " 秒。" << std::endl;

	// step2 使用PLICP计算雷达前后两帧间的坐标变换
	start_time_ = std::chrono::steady_clock::now();

	ScanMatchWithPLICP(curr_ldp_scan, current_time_);

	end_time_ = std::chrono::steady_clock::now();
	time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
	// std::cout << "整体函数处理用时: " << time_used_.count() << " 秒。" << std::endl;
}

/**
 * 雷达数据间的角度是固定的，因此可以将对应角度的cos与sin值缓存下来，不用每次都计算
 */
void ScanMatchPLICP::CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
	a_cos_.clear();
	a_sin_.clear();
	double angle;

	for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
	{
		angle = scan_msg->angle_min + i * scan_msg->angle_increment;
		a_cos_.push_back(cos(angle));
		a_sin_.push_back(sin(angle));
	}

	input_.min_reading = scan_msg->range_min;
	input_.max_reading = scan_msg->range_max;
}

/**
 * 获取机器人坐标系与雷达坐标系间的坐标变换
 */
bool ScanMatchPLICP::GetBaseToLaserTf(const std::string &frame_id)
{
	ros::Time t = ros::Time::now();

	geometry_msgs::TransformStamped transformStamped;
	// 获取tf并不是瞬间就能获取到的，要给1秒的缓冲时间让其找到tf
	try
	{
		transformStamped = tfBuffer_.lookupTransform(base_frame_, frame_id,
																								 t, ros::Duration(1.0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
		ros::Duration(1.0).sleep();
		return false;
	}

	// 将获取的tf存到base_to_laser_中
	tf2::fromMsg(transformStamped.transform, base_to_laser_);
	laser_to_base_ = base_to_laser_.inverse();

	return true;
}

/**
 * 将雷达的数据格式转成 csm 需要的格式
 */
void ScanMatchPLICP::LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp)
{
	unsigned int n = scan_msg->ranges.size();
	ldp = ld_alloc_new(n);

	for (unsigned int i = 0; i < n; i++)
	{
		// 在雷达坐标系下计算位置
		double r = scan_msg->ranges[i];

		if (r > scan_msg->range_min && r < scan_msg->range_max)
		{
			// 填充雷达数据
			ldp->valid[i] = 1;
			ldp->readings[i] = r;
		}
		else
		{
			ldp->valid[i] = 0;
			ldp->readings[i] = -1; // 无效范围设置为-1
		}

		ldp->theta[i] = scan_msg->angle_min + i * scan_msg->angle_increment;
		ldp->cluster[i] = -1;
	}

	ldp->min_theta = ldp->theta[0];
	ldp->max_theta = ldp->theta[n - 1];

	ldp->odometry[0] = 0.0;
	ldp->odometry[1] = 0.0;
	ldp->odometry[2] = 0.0;

	ldp->true_pose[0] = 0.0;
	ldp->true_pose[1] = 0.0;
	ldp->true_pose[2] = 0.0;
}

/**
 * 使用PLICP进行帧间位姿的计算
 */
void ScanMatchPLICP::ScanMatchWithPLICP(LDP &curr_ldp_scan, const ros::Time &time)
{
	// CSM的使用方式如下：
	// 扫描始终在激光帧中
	// 参考扫描（prevLDPcan_）的姿态为[0，0，0]
	// 新的扫描（currLDPScan）具有与移动相等的姿势
	// 自上次扫描以来激光帧中激光的
	// 然后使用tf机械传播计算的校正

	prev_ldp_scan_->odometry[0] = 0.0;
	prev_ldp_scan_->odometry[1] = 0.0;
	prev_ldp_scan_->odometry[2] = 0.0;

	prev_ldp_scan_->estimate[0] = 0.0;
	prev_ldp_scan_->estimate[1] = 0.0;
	prev_ldp_scan_->estimate[2] = 0.0;

	prev_ldp_scan_->true_pose[0] = 0.0;
	prev_ldp_scan_->true_pose[1] = 0.0;
	prev_ldp_scan_->true_pose[2] = 0.0;

	input_.laser_ref = prev_ldp_scan_;
	input_.laser_sens = curr_ldp_scan;

	// 匀速模型，速度乘以时间，得到预测的odom坐标系下的位姿变换
	double dt = (time - last_icp_time_).toSec();
	double pr_ch_x, pr_ch_y, pr_ch_a;
	GetPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);

	tf2::Transform prediction_change;
	CreateTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, prediction_change);

	// 说明固定框架中自上一次kf以来的变化
	//  将odom坐标系下的坐标变换 转换成 base_in_odom_keyframe_坐标系下的坐标变换
	prediction_change = prediction_change * (base_in_odom_ * base_in_odom_keyframe_.inverse());

	// 激光帧中激光位置的预测变化
	// 将base_link坐标系下的坐标变换 转换成 雷达坐标系下的坐标变换
	tf2::Transform prediction_change_lidar;
	prediction_change_lidar = laser_to_base_ * base_in_odom_.inverse() * prediction_change * base_in_odom_ * base_to_laser_;

	// 获取到预测变化量
	input_.first_guess[0] = prediction_change_lidar.getOrigin().getX();
	input_.first_guess[1] = prediction_change_lidar.getOrigin().getY();
	input_.first_guess[2] = tf2::getYaw(prediction_change_lidar.getRotation());

	// ROS_INFO("pr_x %f,pr_y %f,pr_yaw: %f,yaw: %f ",pr_ch_x, pr_ch_y, pr_ch_a,local.yaw);

	// 如果它们是非零的，则自由协方差gsl矩阵以避免内存泄漏
	if (output_.cov_x_m)
	{
		gsl_matrix_free(output_.cov_x_m);
		output_.cov_x_m = 0;
	}
	if (output_.dx_dy1_m)
	{
		gsl_matrix_free(output_.dx_dy1_m);
		output_.dx_dy1_m = 0;
	}
	if (output_.dx_dy2_m)
	{
		gsl_matrix_free(output_.dx_dy2_m);
		output_.dx_dy2_m = 0;
	}

	start_time_ = std::chrono::steady_clock::now();
	// 调用csm进行plicp计算
	sm_icp(&input_, &output_);

	end_time_ = std::chrono::steady_clock::now();
	time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
	// std::cout << "PLICP计算用时: " << time_used_.count() << " 秒。" << std::endl;

	tf2::Transform corr_ch;

	if (output_.valid)
	{
		// 雷达坐标系下的坐标变换
		tf2::Transform corr_ch_l;
		CreateTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

		corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;

		// 更新 base_link 在 odom 坐标系下 的坐标
		base_in_odom_ = base_in_odom_keyframe_ * corr_ch;
	}
	else
	{
		ROS_WARN("not Converged");
	}

	// 发布tf与odom话题
	PublishTFAndOdometry();

	// 检查是否需要更新关键帧坐标
	if (NewKeyframeNeeded(corr_ch))
	{
		// 更新关键帧坐标
		ld_free(prev_ldp_scan_);
		prev_ldp_scan_ = curr_ldp_scan;
		base_in_odom_keyframe_ = base_in_odom_;
	}
	else
	{
		ld_free(curr_ldp_scan);
	}

	last_icp_time_ = time;
}

/**
 * 推测从上次icp的时间到当前时刻间的坐标变换
 * 使用匀速模型，根据当前的速度，乘以时间，得到推测出来的位移
 */
void ScanMatchPLICP::GetPrediction(double &prediction_change_x,
																	 double &prediction_change_y,
																	 double &prediction_change_angle,
																	 double dt)
{
	// 速度绝对值小于 1e-6 , 则认为是静止的
	prediction_change_x = abs(latest_velocity_.linear.x) < 1e-2 ? 0.0 : dt * latest_velocity_.linear.x;
	prediction_change_y = abs(latest_velocity_.linear.y) < 1e-2 ? 0.0 : dt * latest_velocity_.linear.y;
	prediction_change_angle = abs(latest_velocity_.angular.z) < 1e-2 ? 0.0 : dt * latest_velocity_.angular.z;

	if (prediction_change_angle >= M_PI)
		prediction_change_angle -= 2.0 * M_PI;
	else if (prediction_change_angle < -M_PI)
		prediction_change_angle += 2.0 * M_PI;
}

/**
 * 从x,y,theta创建tf
 */
void ScanMatchPLICP::CreateTfFromXYTheta(double x, double y, double theta, tf2::Transform &t)
{
	t.setOrigin(tf2::Vector3(x, y, 0.0));
	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, theta);
	t.setRotation(q);
}

/**
 * 发布tf与odom话题
 */
void ScanMatchPLICP::PublishTFAndOdometry()
{
	geometry_msgs::TransformStamped tf_msg;
	tf_msg.header.stamp = current_time_;
	tf_msg.header.frame_id = odom_frame_;
	tf_msg.child_frame_id = base_frame_;
	tf_msg.transform = tf2::toMsg(base_in_odom_);

	// 发布 odom 到 base_link 的 tf
	tf_broadcaster_.sendTransform(tf_msg);

	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = current_time_;
	odom_msg.header.frame_id = odom_frame_;
	odom_msg.child_frame_id = base_frame_;
	tf2::toMsg(base_in_odom_, odom_msg.pose.pose);
	odom_msg.twist.twist = latest_velocity_;

	// 发布 odomemtry 话题
	odom_publisher_.publish(odom_msg);
}

/**
 * 如果平移大于阈值，角度大于阈值，则创新新的关键帧
 * @return 需要创建关键帧返回true, 否则返回false
 */
bool ScanMatchPLICP::NewKeyframeNeeded(const tf2::Transform &d)
{
	scan_count_++;

	if (fabs(tf2::getYaw(d.getRotation())) > kf_dist_angular_)
		return true;

	if (scan_count_ == kf_scan_count_)
	{
		scan_count_ = 0;
		return true;
	}

	double x = d.getOrigin().getX();
	double y = d.getOrigin().getY();
	// 按照欧氏距离进行计算
	if (x * x + y * y > kf_dist_linear_sq_)
		return true;

	return false;
}

