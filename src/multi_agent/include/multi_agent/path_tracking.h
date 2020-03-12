#ifndef PATH_TRACKING_H_
#define PATH_TRACKING_H_
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
// #include "multi_agent/group_global_goal.h"
// #include "multi_agent/data.h"
#include <nav_msgs/Odometry.h>
#include "nav_msgs/Path.h"
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <tf/transform_listener.h>
namespace multi_agent{
class Path_tracking
{
public:
    Path_tracking();
    void get_Lfw(const double& currant_vel);
    bool get_local_goal(const PoseSE2& currant_odom_pose);
    PoseSE2 execute_loop(const double& currant_vel,const PoseSE2& currant_odom_pose);
    void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
    double get_distance(geometry_msgs::PoseStamped& p1,PoseSE2 p2);
private:
    double Lfw;
    PoseSE2 local_goal;
    nav_msgs::Path map_path, odom_path;
    ros::Subscriber  path_sub;
    ros::NodeHandle n;
    tf::TransformListener tf_listener;
};
}
#endif