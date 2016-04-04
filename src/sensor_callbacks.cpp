#include "babs_slam.h"

void babs_slam::initializeSubscribers(){
	encoder_listener= nh_.subscribe("/odom",1,&babs_slam::encoder_callback,this);
	imu_listener= nh_.subscribe("/imu",1,&babs_slam::imu_callback,this);
	gps_listener= nh_.subscribe("/gps_fix",1,&babs_slam::gps_callback,this);
	lidar_listener= nh_.subscribe("/scan",1,&babs_slam::lidar_callback,this);
}

void babs_slam::initializePublishers(){
	map_publisher = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
}


void babs_slam::warmCallbacks(){
	last_imu_used = last_imu;
	//also gps stuff
}



void babs_slam::encoder_callback(const nav_msgs::Odometry& odom_value){

	double x = odom_value.pose.pose.position.x;
	double y = odom_value.pose.pose.position.y;

	last_odom = odom_value;

	//TODO do something with the odom values
	ROS_INFO("x,y from odom: %f, %f", x,y);

}

void babs_slam::imu_callback(const sensor_msgs::Imu& imu_data){

	last_imu = imu_data;

}

void babs_slam::gps_callback(const std_msgs::Float64& message_holder){

}

void babs_slam::lidar_callback(const sensor_msgs::LaserScan& laser_scan){

	last_scan = laser_scan;

	// //tricking the map updater;
	// for (int i = 0; i <= (last_scan.angle_max-last_scan.angle_min)/last_scan.angle_increment; i++){
	// 	last_scan.ranges[i]=0.0/0.0;
	// }

	// float angle_min = laser_scan.angle_min;
	// float angle_max = laser_scan.angle_max;
	// float angle_increment = laser_scan.angle_increment;
	// float time_increment = laser_scan.time_increment;
	// float scan_time = laser_scan.scan_time;
	// float range_min = laser_scan.range_min;
	// float range_max = laser_scan.range_max;
	//laser_scan.ranges;
	//laser_scan.intensities;

	// ROS_INFO("angle_min = %f", angle_min);
	// ROS_INFO("angle_max = %f", angle_max);
	// ROS_INFO("angle_increment = %f", angle_increment);
	// ROS_INFO("time_increment = %f", time_increment);
	// ROS_INFO("scan_time = %f", scan_time);
	// ROS_INFO("range_min = %f", range_min);
	// ROS_INFO("range_max = %f", range_max);
}
