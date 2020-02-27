#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include "multi_agent/pose_se2.h"
#include "multi_agent/group_global_goal.h"

using namespace multi_agent;
using namespace Eigen;

class FormationController
{
public:
	FormationController(std::string name);
	void local_goalCB(const geometry_msgs::PoseStamped& goal);
	void global_goalCB(const multi_agent::group_global_goal& goal);
	void agent_poseCB(const geometry_msgs::PoseStamped& pose);
private:
	ros::NodeHandle n;
	ros::Publisher cmd_pub;
	ros::Subscriber local_goal_sub;
	ros::Subscriber group_global_goal_sub;
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
};

FormationController::FormationController(std::string name)
{
	ros::NodeHandle pn("~/" + name);
	pn.param("controller_freq", controller_freq, 20);

	cmd_pub=n.advertise<geometry_msgs::Twist>("/control_output", 10); 
	local_goal_sub=n.subscribe("/local_goal", 1, &FormationController::local_goalCB, this);
	group_global_goal_sub=n.subscribe("/global_goal", 1, &FormationController::global_goalCB, this);
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

void FormationController::agent_poseCB(const geometry_msgs::PoseStamped& pose)
{
	agent_pose=PoseSE2(pose);
}

void FormationController::control_loopCB(const ros::TimerEvent&)
{
	e_p=local_goal-agent_pose;
	de_p=(e_p-last_e_p)*(1.0/controller_freq);//bug警告
	PoseSE2 zero_vec;
	PoseSE2 u=zero_vec-de_p;

	geometry_msgs::Twist vel_pub;
	vel_pub.linear.x=u.x();
	vel_pub.linear.y=u.y();
	vel_pub.angular.z=u.theta();
	cmd_pub.publish(vel_pub);

	last_e_p=e_p;
}

int main(int argc, char *argv[])
{

	// MatrixXd m(3,3);

	ros::init(argc, argv, "FormationControl");
	FormationController FCinstance("test_node");
	ros::spin();
    return 0;
}
