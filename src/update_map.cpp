#include "babs_slam.h"

// table 9.1 from the book

bool comparePoints(point p1, point p2){
	if (p1.x == p2.x && p1.y == p2.y){
		return true;
	}
	else{
		return false;
	}
}

int clip(int n, int lower, int upper) {
  return std::max(lower, std::min(n, upper));
}

std::vector<point> babs_slam::get_points_in_scan(particle p, sensor_msgs::LaserScan scan,int i){
	//ROS_INFO("running get_points_in_scan");
	float offset = 0;//TODO: convert this to transform listener
	float ang = convertPlanarQuat2Phi(p.pose.orientation);
	ang += offset;
	ang += scan.angle_min + scan.angle_increment*i;
	//ROS_INFO("angle=%f", ang);
	std::vector<point> result;
	float res = p.map.info.resolution/2;
	float traveled = 0;
	point init;
	init.x = (p.pose.position.x-p.map.info.origin.position.x)/p.map.info.resolution;
	init.y = (p.pose.position.y-p.map.info.origin.position.y)/p.map.info.resolution;
	result.push_back(init);
	//ROS_INFO("init pose = %f %f", p.pose.position.x, p.pose.position.y);
	//ROS_INFO("init point = %d %d", init.x, init.y);
	float drawdist=scan.ranges[i];
	if(!std::isfinite(scan.ranges[i])){
		drawdist = scan.range_max;
	}else if (scan.ranges[i]<scan.range_min){
		//ROS_INFO("invalid scan %d %f %f %f", i, scan.range_min,scan.range_max ,scan.ranges[i]);

		return result;

	}

	//ROS_INFO("valid scan %f", ang);
	
		while(traveled<std::min(drawdist,scan.range_max)){
			traveled += res;
			float xloc = (p.pose.position.x-p.map.info.origin.position.x)/p.map.info.resolution;

			xloc += traveled*cos(ang)/p.map.info.resolution;

			float yloc = (p.pose.position.y-p.map.info.origin.position.y)/p.map.info.resolution;
			yloc += traveled*sin(ang)/p.map.info.resolution;

			point addthis;
			addthis.x = xloc;
			addthis.y = yloc;
			if (!comparePoints(addthis,result.back())){
				//ROS_INFO("Adding point %d %d", addthis.x, addthis.y);
				result.push_back(addthis);
			}
			//ROS_INFO("Finished processing %f %f, traveled=%f", xloc, yloc, traveled);
		}

	// ROS_INFO("result lenght %d", result.size());
	return result;
}


void babs_slam::updateMap(particle &p){
	//for each scan point
	//ROS_INFO("updating map %f", (last_scan.angle_max-last_scan.angle_min)/last_scan.angle_increment);
	for (int i = 0;i<=(last_scan.angle_max-last_scan.angle_min)/last_scan.angle_increment;i++) {
		std::vector<point> coneSlice = get_points_in_scan(p,last_scan,i);
		//for each point in scan
		//ROS_INFO("am here ");
			for (int j = 0; j < coneSlice.size();j++){
				float priorLogOdds = prob_to_log_odds(DEFAULT_VALUE);//0;//log(priorOcc/(1-priorOcc)); // eq 9.7
				int index = coneSlice[j].x + coneSlice[j].y*p.map.info.width;
				if (index >= p.map.info.width*p.map.info.height){
					continue;
				}
				int lOdd = prob_to_log_odds(p.map.data[index]) + inverseSensorModel(last_scan,i,coneSlice,j) - priorLogOdds;
				//p.map.data[index] = clip(lOdd,0,100);
				p.map.data[index] = log_odds_to_prob(lOdd);
		}
//		return map
	}
	//map_publisher.publish(p.map);
}

//float inverseSensorModel
//basicly if grid is occupied, return positive number;
//if unoccupied, return negative number;


//really cheaty approach
int babs_slam::inverseSensorModel(sensor_msgs::LaserScan scan,int i,std::vector<point> coneSlice,int j){
	if (scan.ranges[i]<scan.range_min){
		return 0;
	}
	if (j==coneSlice.size()-1 && scan.ranges[i]<scan.range_max && std::isfinite(scan.ranges[i])){
		return 1;
	}
	else return -1;
}
//lol


//// table 9.2 from the book
//// Computes the change in each map cell
////
//// x and y reference the cell of interest in the occupancy grid.
//// Measurement is the entire lidar scan
//// Pose is the Pose of the robot
//float inverseSensorModel(int x, int y, Measurement mt, Pose pose) {
//	// Get the center of the cell
//	float x = cellX + 0.5;
//	float y = cellY + 0.5;
//	// Get distance and angle of (cell-pose) vector
//	float r = sqrt((x-pose.x)^2+(y-pose.y)^2);
//	float phi = atan2(x-pose.x, y-pose.y) - pose.theta
//	// Find the closest ray
//	minAngle = 99999
//	k = 0
//	for (i=0; i<len(mt.values); i++) {
//		float angle = getAngle(pose, i);
//		if (angle < minAngle) {
//			minAngle = angle;
//			k = i;
//		}
//	}
//	z = mt.values[k]
//	// Change cell occupancy depending on the measurement and cell coordinates
//	if (r > min(zMax, z+ismAlpha/2) || abs(phi-minAngle)>ismBeta/2)
//		return l0;
//	if (z<zMax && abs(r-z)<ismAlpha/2)
//		return lOcc;
//	return lFree;
//}
