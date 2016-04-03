#include "babs_slam.h"

// Testing code

void sensor_model_test(babs_slam babs) {
	ROS_INFO("Testing sensor model");
	for (float i=0; i<11; i+=0.05) {
		float phit = babs.pHit(i,5);
		float pshort = babs.pShort(i, 5);
		float pmax = babs.pMax(i);
		float prand = babs.pRand(i);
		ROS_INFO("i=%f, phit=%f, pshort=%f, pmax=%f, prand=%f",i,phit, pshort,pmax,prand);
	}
}


void measurement_model_test(babs_slam babs) {
	ROS_INFO("Testing measurement model");
	sensor_msgs::LaserScan mt;
	ros::Time scan_time = ros::Time::now();
	int num_readings = 5;
	double laser_frequency = 40;
	double ranges[num_readings];
	ranges[0] = 0.15;
	ranges[1] = 0.21;
	ranges[2] = 0.25;
	ranges[3] = 0.21;
	ranges[4] = 0.15;
	mt.header.stamp = scan_time;
	mt.header.frame_id = "laser_frame";
	mt.angle_min = -1.57;
	mt.angle_max = 1.57;
	mt.angle_increment = 3.14 / (num_readings-1);
	mt.time_increment = (1 / laser_frequency) / (num_readings);
	mt.range_min = 0.0;
	mt.range_max = 8.1;
	mt.ranges.resize(num_readings);
	for(int i = 0; i < num_readings; ++i){
		mt.ranges[i] = ranges[i];
	}

	geometry_msgs::Pose pose;
	pose.position.x = 2.5;
	pose.position.y = 1.5;
	pose.position.z = 0;
	pose.orientation = babs.convertPlanarPhi2Quaternion(M_PI/2.0);

	nav_msgs::OccupancyGrid map;
	int data[] = {0,0,0,0,0, 90,0,0,0,90, 0,0,0,0,0, 90,90,0,90,90, 0,0,90,0,0};
	map.header.stamp = scan_time;
	map.header.frame_id = "map_frame";
	map.info.map_load_time = scan_time;
	map.info.resolution = 0.1;
	map.info.width = 5;
	map.info.height = 5;
	map.data.resize(25);
	for(int i = 0; i < 25; i++){
		map.data[i] = data[i];
	}

	float test = babs.measurementModelMap(mt, pose, map);
	ROS_INFO("Weight: %f", test);

	babs.map_publisher.publish(map);

}

void motion_model_test(babs_slam babs) {
	ROS_INFO("Testing motion model");

	nav_msgs::Odometry state;

	geometry_msgs::Pose pose;
	pose.position.x = 2.5;
	pose.position.y = 1.5;
	pose.position.z = 0;
	pose.orientation = babs.convertPlanarPhi2Quaternion(M_PI/2.0);
	ROS_INFO("Starting pos (%f, %f), orient %f", pose.position.x, pose.position.y, babs.convertPlanarQuat2Phi(pose.orientation));

	geometry_msgs::Twist twist;
	twist.linear.x = 0.1; // forward (in m/s)
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = M_PI/2; //>0 - left, <0 - right

	state.pose.pose = pose;
	state.twist.twist = twist;

	double params[] = {0.001, 0.00001, 0.001, 0.00001, 0.001, 0.00001};

	geometry_msgs::Pose result;
	result = babs.sampleMotionModel( state, params);
	ROS_INFO("Finished pos (%f, %f), orient %f", result.position.x, result.position.y, babs.convertPlanarQuat2Phi(result.orientation));
	result = babs.sampleMotionModel( state, params);
	ROS_INFO("Finished pos (%f, %f), orient %f", result.position.x, result.position.y, babs.convertPlanarQuat2Phi(result.orientation));
	result = babs.sampleMotionModel( state, params);
	ROS_INFO("Finished pos (%f, %f), orient %f", result.position.x, result.position.y, babs.convertPlanarQuat2Phi(result.orientation));
}
void update_map_test(babs_slam babs){
	
	
}

int main(int argc, char** argv)
{

	ROS_INFO("Running babslam_main...");

	srand (static_cast <unsigned> (time(0)));
	ros::init(argc, argv, "babs_slam");

	ros::NodeHandle nh_;

	babs_slam babs(&nh_);
	ROS_INFO("hi");
	//sensor_model_test(babs);
	//measurement_model_test(babs);
	//motion_model_test(babs);
	ros::Rate naptime(1.0);
	while(ros::ok()){
		babs.update();
		ros::spinOnce();
		naptime.sleep();

	}


    ros::spin();
    return 0;
} 
