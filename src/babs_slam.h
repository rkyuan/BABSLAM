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
struct point{
	int x;
	int y;
};

class babs_slam
{
public:
	babs_slam(ros::NodeHandle* nodehandle);

	double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
	geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

	geometry_msgs::Pose sampleMotionModel(nav_msgs::Odometry state, double params[6]);

	float measurementModelMap(sensor_msgs::LaserScan mt, geometry_msgs::Pose pose, nav_msgs::OccupancyGrid map);
	float pHit(float z, float trueZ);
	float pShort(float z, float trueZ);
	float pMax(float z);
	float pRand(float z);
	sensor_msgs::LaserScan raytrace(sensor_msgs::LaserScan mt, geometry_msgs::Pose pose, nav_msgs::OccupancyGrid map);
	float raytrace(double x0, double y0, double x1, double y1, nav_msgs::OccupancyGrid map);

	int map_get_value(nav_msgs::OccupancyGrid map, int x, int y);
	bool within_map_bounds(int x, int y);
	float prob_to_log_odds(int prob);
	int log_odds_to_prob(float logOdds);

	ros::Publisher map_publisher;


	void update();



	void encoder_callback(const nav_msgs::Odometry& odom_value);
	void lidar_callback(const sensor_msgs::LaserScan& laser_scan);
	void imu_callback(const sensor_msgs::Imu& imu_data);
	//TODO: GPS
	void gps_callback(const std_msgs::Float64& message_holder);

private:


	

	//private class variables

	double sample_normal(double bSquared);


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





	const int NUMPARTICLES = 1;
	// Measurement model parameters
	const float Z_HIT = 0.6;
	const float Z_SHORT = 0.1;
	const float Z_MAX = 0.1;
	const float Z_RAND = 0.2;
	const float STD_HIT = 0.05;
	const float L_SHORT = 0.1;
	const float MAX_LIDAR_RANGE = 8.1;
	// Map parameters
	const int MAP_MAX_X = 500;
	const int MAP_MAX_Y = 500;
	const int ROBOT_START_OFFSET_X = 0;
	const int ROBOT_START_OFFSET_Y = 0;
	const float ROBOT_START_ORIENTATION = 0;
	const float MAP_RESOLUTION = 0.1; // meters per cell
	const int MAP_OCC_THRESH = 50; // min occupancy probability to consider a cell occupied
	const int DEFAULT_VALUE = 50; // map occupancy value by default
	  

	void initializeSubscribers();
	void initializePublishers();
	void initializeParticles();

	

	void updateMap(particle &p);
	std::vector<point> get_points_in_scan(particle p, sensor_msgs::LaserScan scan,int i);
	int inverseSensorModel(sensor_msgs::LaserScan scan,int i,std::vector<point> coneSlice,int j);

	void resample(std::vector<float> weights);





	bool compareFloats(float a, float b);
	
	//TODO use a timer or something instead of hard coding
	double dt = 0.1;// time since last running of SLAM,

};

#endif
