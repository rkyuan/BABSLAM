#ifndef BABS_SLAM_H_
#define BABS_SLAM_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>

class babs_slam
{
public:
	babs_slam(ros::NodeHandle* nodehandle);


private:

	geometry_msgs::Pose sampleMotionModel(nav_msgs::Odometry state, double params[]);
	double sample_normal(double bSquared);
	double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
	geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);




	ros::NodeHandle nh_;

	//TODO use a timer or something instead of hard coding
	double dt = 0.1;// time since last running of SLAM,

};

#endif
