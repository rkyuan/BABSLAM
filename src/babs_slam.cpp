#include "babs_slam.h"


babs_slam::babs_slam(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    
}

void babs_slam::update(){
	std::vector<geometry_msgs::Pose> newParticles;
	std::vector<float> particleWeights;
	//result will update particles
	for (int i = 0; i < particles.size();i++){
		geometry_msgs::Pose p = particles[i];
		//get new particles
		geometry_msgs::Pose newpose = sampleMotionModel(p);
		//weigh particles
		particleWeights.push_back(measurementModelMap(newpose));
		updateMap();
	}
	particles=newParticles;
	resample(particleWeights);
}

geometry_msgs::Pose babs_slam::sampleMotionModel(geometry_msgs::Pose p){

	return p;
}

float babs_slam::measurementModelMap(geometry_msgs::Pose p){
	return 0.0;
}

void babs_slam::updateMap(){

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
	std::vector<geometry_msgs::Pose> newParticles;
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

int main(int argc, char** argv) 
{
    

    ros::init(argc, argv, "babs_slam"); 

    ros::NodeHandle nh; 

    
    babs_slam babs(&nh); 

    
    ros::spin();
    return 0;
} 