#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>

class FormationController
{
public:
	FormationController();
	void goalCB(const geometry_msgs::PoseStamped& local_goal);
private:
	ros::NodeHandle n;
	ros::Publisher cmd_pub;
	ros::Subscriber local_goal_sub;
	ros::Time current_time, last_time;
};

FormationController::FormationController()
{
	cmd_pub=n.advertise<geometry_msgs::Twist>("/control_output", 10); 
	local_goal_sub=n.subscribe("/local_goal", 1, &FormationController::goalCB, this);

}

void FormationController::goalCB(const geometry_msgs::PoseStamped& local_goal)
{

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "FormationControl");
	FormationController FCinstance;
	ros::spin();
    return 0;
}