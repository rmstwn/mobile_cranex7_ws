#include <ros/ros.h>
#include <mobile_control/dxl_control.h>
#include <std_msgs/Float64.h>

std_msgs::Float64 v1, v2, v3, v4, v5;

void filterVelocityCallback(const trajectory_msgs::JointTrajectory &traj)
{
    // Using the callback function just for subscribing
    // Subscribing the message and storing it in 'linx' and 'angZ'
    trajectory_msgs::JointTrajectoryPoint point;

    v1.data = traj.points[0].velocities[0];
    v2.data = traj.points[0].velocities[1];
    v3.data = traj.points[0].velocities[2];
    v4.data = traj.points[0].velocities[3];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_velocity");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/drives/joint_trajectory", 1, &filterVelocityCallback);
    // ros::Rate rate(1);

    while (ros::ok())
    {
        // It would be better to apply the conditions within the main function and use the
        // Callback function just for subscribing

        ROS_INFO_STREAM("Subscriber velocities:"
                        << " V1=" << v1.data << " V2=" << v2.data << " V3=" << v3.data << " V3=" << v4.data);
        // The above line doesn't publish. It's like printf (but not exactly)

        ros::spinOnce();
    }
}