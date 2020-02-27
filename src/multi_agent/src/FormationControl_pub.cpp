#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include "multi_agent/pose_se2.h"

using namespace multi_agent;

class FormationController
{
public:
	FormationController(std::string name);
	void local_goalCB(const geometry_msgs::PoseStamped& goal);
	void global_goalCB(const geometry_msgs::PoseStamped& goal);
	void agent_poseCB(const geometry_msgs::PoseStamped& pose);
private:
	ros::NodeHandle n;
	ros::Publisher cmd_pub;
	ros::Subscriber local_goal_sub;
	ros::Subscriber global_goal_sub;
	ros::Time current_time, last_time;
	PoseSE2 agent_pose;
	PoseSE2 local_goal;
	PoseSE2 global_goal;
	// geometry_msgs::PoseStamped local_goal;
	// geometry_msgs::PoseStamped global_goal;
	// geometry_msgs::PoseStamped robot_pose;

	int controller_freq;
	ros::Timer timer;
	void control_loopCB(const ros::TimerEvent&);

	PoseSE2 e_p;
};

FormationController::FormationController(std::string name)
{
	ros::NodeHandle pn("~/" + name);
	pn.param("controller_freq", controller_freq, 20);



	cmd_pub=n.advertise<geometry_msgs::Twist>("/control_output", 10); 
	local_goal_sub=n.subscribe("/local_goal", 1, &FormationController::local_goalCB, this);
	global_goal_sub=n.subscribe("/global_goal", 1, &FormationController::global_goalCB, this);
	timer = n.createTimer(ros::Duration((1.0)/controller_freq), &FormationController::control_loopCB, this);
}

void FormationController::local_goalCB(const geometry_msgs::PoseStamped& goal)
{
	local_goal=PoseSE2(goal);
}

void FormationController::global_goalCB(const geometry_msgs::PoseStamped& goal)
{
	global_goal=PoseSE2(goal);
}

void FormationController::agent_poseCB(const geometry_msgs::PoseStamped& pose)
{
	agent_pose=PoseSE2(pose);
}

void FormationController::control_loopCB(const ros::TimerEvent&)
{
	e_p=local_goal-agent_pose;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "FormationControl");
	FormationController FCinstance("test_node");
	ros::spin();
    return 0;
}
