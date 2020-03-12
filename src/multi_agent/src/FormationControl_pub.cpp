#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <iostream>
#include "multi_agent/pose_se2.h"
#include "multi_agent/group_global_goal.h"
#include "multi_agent/data.h"
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include "multi_agent/path_tracking.h"

// using namespace multi_agent;
using namespace Eigen;
namespace multi_agent{
class FormationController
{
public:
	FormationController(std::string name);
	void local_goalCB(const geometry_msgs::PoseStamped& goal);
	void global_goalCB(const multi_agent::group_global_goal& goal);
	void agent_poseCB(const nav_msgs::Odometry::ConstPtr& pose,const int& id);
	void agent_poseCB(const nav_msgs::Odometry& pose);
	PoseSE2 calculate_interaction_sum();
	PoseSE2 calculate_center_pose();
private:
	ros::NodeHandle n;
	ros::Publisher cmd_pub;
	ros::Publisher data_pub;
	ros::Publisher center_odom_pub;;
	ros::Subscriber local_goal_sub;
	ros::Subscriber group_global_goal_sub;
	std::vector<ros::Subscriber> agent_pose_subs;
	ros::Subscriber sub_test;
	PoseSE2 agent_pose;
	nav_msgs::Odometry agent_pose_nav;
	PoseSE2 local_goal;
	MatrixXd group_global_goal;
	MatrixXd all_agent_poses;

	boost::shared_ptr<Path_tracking> path_tracking_ptr_;

	int controller_freq;
	ros::Timer timer;
	void control_loopCB(const ros::TimerEvent&);

	PoseSE2 e_p;
	PoseSE2 de_p;
	PoseSE2 zero_vec;
	PoseSE2 u;
	double k_p;

	double test_goal_x;
	double test_goal_y;

	std::string agent_name;
	std::string agent_id_str;
	int agent_id;
	int group_size;
	int lock_agent_pose_subs_init=0;

	multi_agent::data data;
};
FormationController::FormationController(std::string name)
{
	// 加载参数 loading parameters
	ros::NodeHandle pn("~");
	pn.param("controller_freq", controller_freq, 10);
	pn.param("k_p", k_p, 2.0);
	pn.param("test_goal_x", test_goal_x, 5.0);
	pn.param("test_goal_y", test_goal_y, 5.0);
	pn.param("group_size", group_size, 1);
	pn.param<std::string>("agent_name", agent_name, "robot");
	//初始化存储全局目标和编队位置的矩阵 Initializes the matrix that stores the destination and location
	group_global_goal.resize(3,group_size);
	all_agent_poses.resize(3,group_size);
	//这是一个智能指针，用于循迹 This is a pointer for path tracking
	path_tracking_ptr_ = boost::shared_ptr<Path_tracking>(new Path_tracking);
	// 初始化订阅者和发布者 Initializes subscribers and publishers
	cmd_pub=n.advertise<geometry_msgs::Twist>(agent_name+"/cmd_vel", 10); 
	data_pub=n.advertise<multi_agent::data>(agent_name+"/data", 10); 
	center_odom_pub=n.advertise<nav_msgs::Odometry>("/center_odom", 10); 
	local_goal_sub=n.subscribe("/local_goal", 1, &FormationController::local_goalCB, this);
	group_global_goal_sub=n.subscribe("/global_goal", 1, &FormationController::global_goalCB, this);
	agent_name.erase(0,6);
	agent_id=std::atoi(agent_name.c_str());
	ROS_WARN("agent_id:%d",agent_id);
	agent_pose_subs.push_back(n.subscribe<nav_msgs::Odometry>("/agent"+ std::to_string(agent_id)+"/odom", 1,boost::bind(&FormationController::agent_poseCB,this, _1,agent_id)));
	//控制循环 control loop
	timer = n.createTimer(ros::Duration((1.0)/controller_freq), &FormationController::control_loopCB, this);
	ros::Duration(5.0).sleep();
}
void FormationController::local_goalCB(const geometry_msgs::PoseStamped& goal)
{//局部目标的回调函数（弃用） Callback function for local target (deprecated)
	local_goal=PoseSE2(goal);
}
void FormationController::global_goalCB(const multi_agent::group_global_goal& goal)
{//全局目标的回调函数
	group_size=goal.id.size();
	for(int each_id:goal.id)
	{	//创建用于储存全局目标的矩阵 build group_global_goal Matrix
		RowVectorXd agent_state_vec(3);
		agent_state_vec << goal.poses[each_id].pose.position.x,goal.poses[each_id].pose.position.y,goal.poses[each_id].pose.orientation.z;
		group_global_goal.col(each_id)=agent_state_vec;
		//订阅邻居的位置  subscribe to neighbour's pose
		if(each_id!=agent_id&&lock_agent_pose_subs_init<(group_size-1))
		{
			agent_pose_subs.push_back(n.subscribe<nav_msgs::Odometry>("/agent"+ std::to_string(each_id)+"/odom", 1, boost::bind(&FormationController::agent_poseCB,this, _1,each_id)));
			lock_agent_pose_subs_init++;//subscriber只需要初始化一次，避免重复初始化
		}
	}
}
void FormationController::agent_poseCB(const nav_msgs::Odometry& pose)
{//（弃用）(deprecated)
	agent_pose=PoseSE2(pose.pose.pose);
}
void FormationController::agent_poseCB(const nav_msgs::Odometry::ConstPtr& pose,const int& id)
{//邻居（包括自己）的位置的回调函数 The callback function for the location of the neighbors (including currant robot)
	nav_msgs::Odometry _pose=*pose;
	//机器人自身的位置 
	if(id==agent_id)
	{
		agent_pose=PoseSE2(_pose.pose.pose);
		agent_pose_nav=_pose;
	}
	//邻居的位置neighbour's pose
	PoseSE2 a_p=PoseSE2(_pose.pose.pose);
	all_agent_poses.col(id)<<a_p.x(),a_p.y(),a_p.theta();
}
void FormationController::control_loopCB(const ros::TimerEvent&)
{//主循环 main loop
	//计算中点 calculate center pose
	PoseSE2 center_pose=calculate_center_pose();
	//发布中点，move_base将会以中点为起点规划路径 publish it.move_base would use it for Path planning.
	nav_msgs::Odometry center_odom;
	center_odom=agent_pose_nav;
	center_odom.pose.pose.position.x=center_pose.x();
	center_odom.pose.pose.position.y=center_pose.y();
	center_odom_pub.publish(center_odom);
	//计算局部目标点 Calculate the local target point      \bar{p}^*
	PoseSE2 bar_p_star_se2=path_tracking_ptr_->execute_loop(2,center_pose);
	//计算分布  Calculate distribution                     p^*=\bar{p}^* E+\mathfrac{p}^*-1/N \mathfrac{p}^* E^T E
	MatrixXd E=MatrixXd::Ones(1,group_size);
	MatrixXd E_T=E.transpose();
	MatrixXd bar_p_star(3,1);
	bar_p_star<<bar_p_star_se2.x(),bar_p_star_se2.y(),bar_p_star_se2.theta();
	MatrixXd mathfrac_p_star=group_global_goal;
	MatrixXd p_star=bar_p_star*E+mathfrac_p_star-(mathfrac_p_star*E_T*E)/group_size;
	local_goal=PoseSE2(p_star(0,agent_id),p_star(1,agent_id),p_star(2,agent_id));
	//计算控制输出 Calculate control output                u=k_p*e_p+k_p*\sum_{mathfrak{N}_i}{\omega}_{ij}{p_j-p_i-p_j^*+p_i^*}
	e_p=local_goal-agent_pose;
	PoseSE2 interaction_sum=calculate_interaction_sum();
	u=k_p*e_p+k_p*interaction_sum;
	//发布控制输出 publish control output
	geometry_msgs::Twist vel_pub;
	vel_pub.linear.x=u.x();
	vel_pub.linear.y=u.y();
	vel_pub.angular.z=u.theta();
	cmd_pub.publish(vel_pub);
	//发布误差 publish error
	data.formation_err=e_p.length();
	data.group_err=interaction_sum.length();
	data_pub.publish(data);
}
PoseSE2 FormationController::calculate_interaction_sum()
{//  \sum_{mathfrak{N}_i}{\omega}_{ij}{p_j-p_i-p_j^*+p_i^*}
	PoseSE2 sum;
	for(int i=0;i<group_size;i++)
	{	
		//排除自身
		if(i==agent_id)
			continue;
		//B=p_j^*-p_i^*
		MatrixXd B=group_global_goal.col(i)-group_global_goal.col(agent_id);
		//A=p_j-p_i
		MatrixXd A=all_agent_poses.col(i)-all_agent_poses.col(agent_id);
		//S=A-B
		MatrixXd S=A-B;
		sum+=PoseSE2(S(0,0),S(1,0),S(2,0));
	}
	return sum;
}
PoseSE2 FormationController::calculate_center_pose()
{//计算编队的中点 Calculate the midpoint of the formation
	PoseSE2 center;
	double x;
	double y;
	double theta;
	for(int i=0;i<group_size;i++)
	{
		x+=all_agent_poses(0,i);
		y+=all_agent_poses(1,i);
		theta+=all_agent_poses(2,i);
	}
	center=PoseSE2(x/group_size,y/group_size,theta/group_size);
	return center;
}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "FormationControl");
	multi_agent::FormationController FCinstance("test_node");
	ros::spin();
    return 0;
}
