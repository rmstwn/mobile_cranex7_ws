#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

typedef struct {
	double xAbs;
	double yAbs;
	double phiAbs;
} OdomPose;

class Kinematics {
public:
	virtual void execForwKin(const sensor_msgs::JointState& js, nav_msgs::Odometry& odom, OdomPose& cpose) {};
	virtual void execInvKin(const geometry_msgs::Twist& twist, trajectory_msgs::JointTrajectory& traj) {};

protected:
	ros::Time current_time, last_time;
	nav_msgs::Odometry last_odom;

};


#endif // KINEMATICS_H_
