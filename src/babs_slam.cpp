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
	std::vector<particle> newParticles;
	std::vector<float> particleWeights;
	//result will update particles
	for (int i = 0; i < particles.size();i++){
		geometry_msgs::Pose p = particles[i].pose;
		//get new particles
		geometry_msgs::Pose newpose = last_odom.pose.pose;
		newpose.position.x = newpose.position.x + ROBOT_START_POSE_X;
		newpose.position.y = newpose.position.y + ROBOT_START_POSE_Y;
		particles[i].pose = newpose;
		//geometry_msgs::Pose newpose = sampleMotionModel(p);
		//weigh particles
		//float weight = 1;
		//weight *= measurementModelMap(newpose)
		//weight *= imu_model(newpose,pose)
		//particleWeights.push_back(weight);
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








