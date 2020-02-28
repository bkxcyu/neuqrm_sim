#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include "multi_agent/pose_se2.h"
#include "multi_agent/group_global_goal.h"
#include <nav_msgs/Odometry.h>

using namespace multi_agent;
using namespace Eigen;

class FormationController
{
public:
	FormationController(std::string name);
	void local_goalCB(const geometry_msgs::PoseStamped& goal);
	void global_goalCB(const multi_agent::group_global_goal& goal);
	void agent_poseCB(const nav_msgs::Odometry& pose);
private:
	ros::NodeHandle n;
	ros::Publisher cmd_pub;
	ros::Subscriber local_goal_sub;
	ros::Subscriber group_global_goal_sub;
	ros::Subscriber agent_pose_sub;
	ros::Time current_time, last_time;
	PoseSE2 agent_pose;
	PoseSE2 local_goal;
	MatrixXd group_global_goal;
	// geometry_msgs::PoseStamped local_goal;
	// geometry_msgs::PoseStamped global_goal;
	// geometry_msgs::PoseStamped robot_pose;

	int controller_freq;
	ros::Timer timer;
	void control_loopCB(const ros::TimerEvent&);

	PoseSE2 e_p;
	PoseSE2 last_e_p;
	PoseSE2 de_p;
	PoseSE2 zero_vec;
	PoseSE2 u;
	PoseSE2 last_u;
	double k_p;

	double test_goal_x;
	double test_goal_y;

	std::string agent_name;
};

FormationController::FormationController(std::string name)
{
	ros::NodeHandle pn("~");
	pn.param("controller_freq", controller_freq, 10);
	pn.param("k_p", k_p, 2.0);
	pn.param("test_goal_x", test_goal_x, 5.0);
	pn.param("test_goal_y", test_goal_y, -5.0);
	pn.param<std::string>("agent_name", agent_name, "robot");
	// agent_name.erase(0,6);
	ROS_WARN("agent_name:%s",agent_name.c_str());
	// ROS_INFO("controller param:\n controller_freq=%d\n k_p=%f\n goal=(%f,%f)",controller_freq,k_p,test_goal_x,test_goal_y);

	// cmd_pub=n.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 10); 
	cmd_pub=n.advertise<geometry_msgs::Twist>(agent_name+"/cmd_vel", 10); 
	local_goal_sub=n.subscribe("/local_goal", 1, &FormationController::local_goalCB, this);
	group_global_goal_sub=n.subscribe("/global_goal", 1, &FormationController::global_goalCB, this);
	// agent_pose_sub=n.subscribe("/robot_1/odom", 1, &FormationController::agent_poseCB, this);
	agent_pose_sub=n.subscribe(agent_name+"/odom", 1, &FormationController::agent_poseCB, this);
	timer = n.createTimer(ros::Duration((1.0)/controller_freq), &FormationController::control_loopCB, this);

	current_time = ros::Time::now();
    last_time = ros::Time::now();

}

void FormationController::local_goalCB(const geometry_msgs::PoseStamped& goal)
{
	local_goal=PoseSE2(goal);
}

void FormationController::global_goalCB(const multi_agent::group_global_goal& goal)
{
	group_global_goal.resize(3,goal.id.size());
	for(int each_id:goal.id)
	{
		RowVectorXd agent_state_vec(3);
		agent_state_vec << goal.poses[each_id].pose.position.x,goal.poses[each_id].pose.position.y,goal.poses[each_id].pose.orientation.z;
		group_global_goal.col(each_id)=agent_state_vec;//col 从0开始还是从1 此处可能会出现bug
	}
}

void FormationController::agent_poseCB(const nav_msgs::Odometry& pose)
{
	agent_pose=PoseSE2(pose.pose.pose);
}

void FormationController::control_loopCB(const ros::TimerEvent&)
{
	// test
	local_goal=PoseSE2(test_goal_x,test_goal_y,0);

	e_p=local_goal-agent_pose;
	// ROS_INFO("agent_pose=(%f,%f,%f)",agent_pose.x(),agent_pose.y(),e_p.theta());
	ROS_INFO("e_p=(%f,%f,%f)",e_p.x(),e_p.y(),e_p.theta());
	de_p=(e_p-last_e_p)*controller_freq;
	// ROS_INFO("de_p=(%f,%f,%f)",de_p.x(),de_p.y(),de_p.theta());
	u=k_p*e_p;
	ROS_INFO("u=(%f,%f,%f)\n",u.x(),u.y(),u.theta());

	geometry_msgs::Twist vel_pub;
	vel_pub.linear.x=u.x();
	vel_pub.linear.y=u.y();
	vel_pub.angular.z=u.theta();
	cmd_pub.publish(vel_pub);

	last_u=u;
	last_e_p=e_p;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "FormationControl");
	FormationController FCinstance("test_node");
	ros::spin();
    return 0;
}
