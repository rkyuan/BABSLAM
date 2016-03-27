#ifndef BABS_SLAM_H_
#define BABS_SLAM_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>


class babs_slam
{
public:
	babs_slam(ros::NodeHandle* nodehandle);


private:
	static const int NUMPARTICLES = 500;
	ros::NodeHandle nh_;
	std::vector<geometry_msgs::Pose> particles;
	nav_msgs::OccupancyGrid map;


	void update();
	geometry_msgs::Pose sampleMotionModel(geometry_msgs::Pose p);
	float measurementModelMap(geometry_msgs::Pose p);
	void updateMap();
	void resample(std::vector<float> weights);



};

#endif
