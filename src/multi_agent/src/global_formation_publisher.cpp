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

class GlobalFormationPublisher
{
public:
    GlobalFormationPublisher();
    void GFref_CB(const geometry_msgs::PoseStamped& goalMsg);
    void testpub_loopCB(const ros::TimerEvent&);
private:
    ros::NodeHandle n;
	ros::Publisher GF_pub;
    ros::Subscriber GFref_sub;
    ros::Timer timer;

    multi_agent::PoseSE2 GFref;
    multi_agent::group_global_goal ggg;

    int group_size;
    bool test_mode;
    int testpub_freq;
};

GlobalFormationPublisher::GlobalFormationPublisher()
{
    ros::NodeHandle pn("~");
    pn.param("group_size",group_size,3);
    pn.param<bool>("test_mode",test_mode,true);
    pn.param("testpub_freq",testpub_freq,1);
    GF_pub=n.advertise<multi_agent::group_global_goal>("/global_goal", 10); 
    GFref_sub=n.subscribe("/move_base_simple/goal", 1, &GlobalFormationPublisher::GFref_CB, this);
    timer = n.createTimer(ros::Duration((1.0)/testpub_freq), &GlobalFormationPublisher::testpub_loopCB, this);
    if(test_mode)
    {
        ggg.id.push_back(0);
        geometry_msgs::PoseStamped pose_spawn;
        pose_spawn.pose.position.x=-0.5;
        pose_spawn.pose.position.y=0.5;
        ggg.poses.push_back(pose_spawn);
        ggg.id.push_back(1);
        pose_spawn.pose.position.x=0.5;
        pose_spawn.pose.position.y=0.5;
        ggg.poses.push_back(pose_spawn);
        ggg.id.push_back(2);
        pose_spawn.pose.position.x=0.5;
        pose_spawn.pose.position.y=-0.5;
        ggg.poses.push_back(pose_spawn);
        // ggg.id.push_back(3);
        // pose_spawn.pose.position.x=-0.5;
        // pose_spawn.pose.position.y=-0.5;
        // ggg.poses.push_back(pose_spawn);
        // for(int i=0;i<group_size;i++)
        // {
        //     ggg.id.push_back(i);
        //     geometry_msgs::PoseStamped pose_spawn;
        //     pose_spawn.pose.position.x=0.5*i-1;
        //     pose_spawn.pose.position.y=0;
        //     ggg.poses.push_back(pose_spawn);
        // }
    }
}

void GlobalFormationPublisher::testpub_loopCB(const ros::TimerEvent&)
{
    if(test_mode)
    {
        GF_pub.publish(ggg);
    }
}

void GlobalFormationPublisher::GFref_CB(const geometry_msgs::PoseStamped& goalMsg)
{

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "global_formation_publisher");
	GlobalFormationPublisher GFPinstance;
	ros::spin();
    return 0;
}
