#include "babs_slam.h"


babs_slam::babs_slam(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor


	ROS_INFO("initializing subscribers");
	initializeSubscribers();
	initializePublishers();
	initializeParticles();
}

void babs_slam::initializeParticles(){
	particles.clear();
	nav_msgs::MapMetaData info;
	info.resolution = MAP_RESOLUTION;
	info.width = MAP_MAX_X;
	info.height = MAP_MAX_Y;

	geometry_msgs::Pose origin;
	origin.position.x = 0;//-30;
	origin.position.y = 0;//-30;

	info.origin = origin;
	ROS_INFO("here");
	geometry_msgs::Pose p;
	p.position.x = ROBOT_START_POSE_X;
	p.position.y = ROBOT_START_POSE_Y;
	p.orientation = convertPlanarPhi2Quaternion(0);
	for (int i = 0; i < NUMPARTICLES; i++){

		nav_msgs::OccupancyGrid map;
		map.header.frame_id = "map_frame";
		map.info = info;
		map.data.resize(MAP_MAX_X*MAP_MAX_Y);
		for (int j = 0; j < MAP_MAX_X*MAP_MAX_Y; j++){
			map.data[j]=50;
		}
		particle part;
		part.pose = p;
		part.map = map;
		particles.push_back(part);
	}
}


//this is the highest level of abstraction for the algorithm
void babs_slam::update(){
	ROS_INFO("updating");
	//THIS IS JUST TESTING---------------------------------------
	std::vector<float> particleWeights;
	//result will update particles
	for (int i = 0; i < particles.size();i++){
		geometry_msgs::Pose p = particles[i].pose;
		//get new particles
		geometry_msgs::Pose newpose = last_odom.pose.pose;
		newpose.position.x = newpose.position.x; + ROBOT_START_POSE_X;
		newpose.position.y = newpose.position.y + ROBOT_START_POSE_Y;
		particles[i].pose = newpose;

		if (i == 0) {

			ROS_INFO("broadcasting transform");


			geometry_msgs::Quaternion odom_quat = newpose.orientation;

			//publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = ros::Time::now();
//			odom_trans.header.frame_id = "base_laser1_link";
//			odom_trans.child_frame_id = "base_link";

			odom_trans.header.frame_id = "base_laser1_link";
			odom_trans.child_frame_id = "map_frame";

//			odom_trans.header.frame_id = "map_frame";
//			odom_trans.child_frame_id = "base_laser1_link";

			odom_trans.transform.translation.x = newpose.position.x;
			odom_trans.transform.translation.y = newpose.position.y - ROBOT_START_POSE_Y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

		}



		//geometry_msgs::Pose newpose = sampleMotionModel(p);
		//weigh particles
		//particleWeights.push_back(measurementModelMap(newpose));
		ROS_INFO("particle pose %f %f", particles[i].pose.position.x, particles[i].pose.position.y);
		updateMap(particles[i]);
	}
	
	//resample(particleWeights);
}


void babs_slam::resample(std::vector<float> weights){
	float total_weight = 0;
	for (int i = 0; i < NUMPARTICLES; i ++){
		//add up the size of all the weights
		total_weight+= weights[i];
	}
	std::vector<float> samples;
	for (int i = 0; i < NUMPARTICLES; i++){
		//take a bunch of random numbers in the range of the weights
		samples.push_back(rand()/RAND_MAX*total_weight);
	}
	std::sort(samples.begin(),samples.end());
	std::vector<particle> newParticles;
	total_weight =  weights[0];
	int weight_counter = 0;
	for (int i = 0; i < NUMPARTICLES; i++){
		if (total_weight>samples[i]){
			newParticles.push_back(particles[weight_counter]);
		}
		else{
			weight_counter ++;
			total_weight += weights[weight_counter];
		}
	}
	particles = newParticles;
}

// a useful conversion function: from quaternion to yaw
double babs_slam::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
	double quat_z = quaternion.z;
	double quat_w = quaternion.w;
	double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
	return phi;
}

//and the other direction:
geometry_msgs::Quaternion babs_slam::convertPlanarPhi2Quaternion(double phi) {
	geometry_msgs::Quaternion quaternion;
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin(phi / 2.0);
	quaternion.w = cos(phi / 2.0);
	return quaternion;
}


bool babs_slam::compareFloats(float a, float b) {
	if (fabs(a-b) < 0.001)
		return true;
	return false;
}

// Map helper functions
// Gets int8 value of the cell (x,y) given row-major representation
int babs_slam::map_get_value(nav_msgs::OccupancyGrid map, int x, int y) {
	int index = x + y*MAP_MAX_X;
	return map.data[index];
}

// Check if (x,y) is within map bounds
bool babs_slam::within_map_bounds(int x, int y) {
	return (0<=x && x<MAP_MAX_X && 0<=y && y<MAP_MAX_Y);
}

// Takes probability 0-100 and returns log odds representation
float babs_slam::prob_to_log_odds(int prob) {
	return log(prob/(100-prob));
}

// Takes log odds representation and returns the probability 0-100
int babs_slam::log_odds_to_prob(float logOdds) {
	return 100*exp(logOdds)/(1+exp(logOdds));
}







