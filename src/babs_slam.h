#ifndef BABS_SLAM_H_
#define BABS_SLAM_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64.h> 
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <vector>


struct particle{
	geometry_msgs::Pose pose;
	nav_msgs::OccupancyGrid map;
};

class babs_slam
{
public:
	babs_slam(ros::NodeHandle* nodehandle);

	

	float pHit(float z, float trueZ);
	float pShort(float z, float trueZ);
	float pMax(float z);
	float pRand(float z);

private:

	geometry_msgs::Pose sampleMotionModel(nav_msgs::Odometry state, double params[]);
	double sample_normal(double bSquared);
	double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
	geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

	
	ros::NodeHandle nh_;
	std::vector<particle> particles;
	nav_msgs::OccupancyGrid map;
	nav_msgs::Odometry last_odom;
	sensor_msgs::LaserScan last_scan;
	sensor_msgs::Imu last_imu;

	ros::Subscriber encoder_listener;
	ros::Subscriber imu_listener;
	ros::Subscriber gps_listener;
	ros::Subscriber lidar_listener;




	const int NUMPARTICLES = 500;
	// Measurement model parameters
	const float Z_HIT = 1;
	const float Z_SHORT = 0;
	const float Z_MAX = 0;
	const float Z_RAND = 0;
	const float STD_HIT = 0.05;
	const float L_SHORT = 0.1;
	const float MAX_LIDAR_RANGE = 8.1;

	void initializeSubscribers();
	void update();
	float measurementModelMap(geometry_msgs::Pose p);

	void updateMap(particle p/* , const sensor_msgs::LaserScan& laser_scan, nav_msgs::OccupancyGrid& map*/);

	void resample(std::vector<float> weights);


	void encoder_callback(const nav_msgs::Odometry& odom_value);
	void lidar_callback(const sensor_msgs::LaserScan& laser_scan);

	//TODO implement callbacks for these with correct message types
	void imu_callback(const sensor_msgs::Imu& imu_data);
	void gps_callback(const std_msgs::Float64& message_holder);



	bool compareFloats(float a, float b);
	
	//TODO use a timer or something instead of hard coding
	double dt = 0.1;// time since last running of SLAM,

};

#endif
