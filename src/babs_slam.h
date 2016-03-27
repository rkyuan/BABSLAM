#ifndef BABS_SLAM_H_
#define BABS_SLAM_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>
#include <vector>


class babs_slam
{
public:
	babs_slam(ros::NodeHandle* nodehandle);


private:

	geometry_msgs::Pose sampleMotionModel(nav_msgs::Odometry state, double params[]);
	double sample_normal(double bSquared);
	double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
	geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

	static const int NUMPARTICLES = 500;
	ros::NodeHandle nh_;
	std::vector<geometry_msgs::Pose> particles;
	nav_msgs::OccupancyGrid map;


	void update();
	geometry_msgs::Pose sampleMotionModel(geometry_msgs::Pose p);
	float measurementModelMap(geometry_msgs::Pose p);
	void updateMap();
	void resample(std::vector<float> weights);



	//TODO use a timer or something instead of hard coding
	double dt = 0.1;// time since last running of SLAM,

};

#endif
