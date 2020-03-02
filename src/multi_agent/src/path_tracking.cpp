#include "multi_agent/path_tracking.h"


namespace multi_agent{

Path_tracking::Path_tracking() 
{
    path_sub = n.subscribe("/move_base/NavfnROS/plan", 1, &Path_tracking::pathCB, this);
}   
void Path_tracking::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    map_path = *pathMsg;
}
void Path_tracking::get_Lfw(const double& currant_vel)
{
    if(currant_vel<1)
        Lfw=0.5;
    if(currant_vel>=1&&currant_vel<=2)
        Lfw=1.0;
    if(currant_vel>2)
        Lfw=2.0; 
}
bool Path_tracking::get_local_goal(const PoseSE2& currant_odom_pose)
{
    bool succeed=false;
    if(map_path.poses.size()==0)
    {
        local_goal=currant_odom_pose;
        return succeed;
    }

    for(geometry_msgs::PoseStamped each_path_point:map_path.poses)
    {
        geometry_msgs::PoseStamped path_point_odom;
        try
        {
            tf_listener.transformPose("odom", ros::Time(0),each_path_point,"map" ,path_point_odom);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            break;
            // ros::Duration(1.0).sleep();
        }
        double dis=currant_odom_pose.distance(PoseSE2(path_point_odom));
        // ROS_INFO("distance:%.2f",dis);
        if(dis>=Lfw)
        {
            local_goal=PoseSE2(path_point_odom);
            ROS_INFO("have caught a path point as local goal");
            succeed=true;
        }
        if(succeed)
            break;
    }
    return succeed;
}
PoseSE2 Path_tracking::execute_loop(const double& currant_vel,const PoseSE2& currant_odom_pose)
{
    get_Lfw(currant_vel);
    if(!get_local_goal(currant_odom_pose))
        ROS_WARN("failed to catch local goal,now use last local goal");
    return local_goal;
}
}
