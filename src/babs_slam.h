#ifndef BABS_SLAM_H_
#define BABS_SLAM_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>


class babs_slam
{
public:
	babs_slam(ros::NodeHandle* nodehandle);


private:
	ros::NodeHandle nh_;
	std::vector<geometry_msgs::Pose> particles;
	void update();
	geometry_msgs::Pose sampleMotionModel(geometry_msgs::Pose p);



};

#endif
