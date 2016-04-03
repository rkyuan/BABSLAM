#include "babs_slam.h"

// Table 6.1 from the book
// Not sure about other sensors. We could probably just use Gaussians to model it.
float babs_slam::measurementModelMap(sensor_msgs::LaserScan mt, geometry_msgs::Pose pose, nav_msgs::OccupancyGrid map) {
	float result = 1;
	int numScans = mt.ranges.size();

	// Ray cast to predict the result of the scan
	sensor_msgs::LaserScan scan;
	scan.angle_min = mt.angle_min;
	scan.angle_max = mt.angle_max;
	scan.angle_increment = mt.angle_increment;
	scan.range_max = mt.range_max;
	scan.header = mt.header;
	scan.header.frame_id = "laser_frame";
	scan.ranges.resize(numScans);
	scan = raytrace(scan, pose, map);

	// Compute the probabilities from the actual scan values
	for (int i=0;i<numScans;i++) {
		float z = mt.ranges[i];
		float trueZ = scan.ranges[i];

		float prob = Z_HIT*pHit(z, trueZ);
		prob += Z_SHORT*pShort(z, trueZ);
		prob += Z_MAX*pMax(z);
		prob += Z_RAND*pRand(z);
		result = result*prob;

		ROS_INFO("i=%d, z=%f, trueZ=%f", i, z, trueZ);
	}
	return result;
}

// eq 6.4-6.6 form the book
float babs_slam::pHit(float z, float trueZ) {
	if (z<0.001 || z>MAX_LIDAR_RANGE)
		return 0;
	float gaussianProb = (1/sqrt(2*M_PI*pow(STD_HIT,2)))*exp(-0.5*pow((z-trueZ),2)/pow(STD_HIT,2));
	float normalizer = 1; // Supposed to be an integral, but it only matters when measured value is close to min or max
	return gaussianProb*normalizer;
}

// eq 6.7-6.9 from the book
float babs_slam::pShort(float z, float trueZ) {
	if (z<0 || z>trueZ || z>MAX_LIDAR_RANGE)
		return 0;
	float expProb = L_SHORT*exp(-L_SHORT*z);
	float normalizer = 1/(1-exp(-L_SHORT*trueZ));
	return expProb*normalizer;
}

// eq 6.10 from the book
float babs_slam::pMax(float z) {
	if (babs_slam::compareFloats(z, MAX_LIDAR_RANGE))
		return 1;
	return 0;
}

// eq 6.11 from the book
float babs_slam::pRand(float z) {
	if (z<0 || z>MAX_LIDAR_RANGE)
		return 0;
	return 1/MAX_LIDAR_RANGE;
}

// Simulates LIDAR scan and modifies the ranges[] array of the input scan
sensor_msgs::LaserScan babs_slam::raytrace(sensor_msgs::LaserScan mt, geometry_msgs::Pose pose, nav_msgs::OccupancyGrid map) {
	//get x0,y0 from pose
	double x0 = pose.position.x;
	double y0 = pose.position.y;

	int numRays = (int)((mt.angle_max-mt.angle_min)/mt.angle_increment)+1;
	mt.ranges.resize(numRays);
	for (int i=0; i<numRays; i++) {
		// get x1,y1 for each ray
		double angle = convertPlanarQuat2Phi(pose.orientation) + mt.angle_min + i*mt.angle_increment;
		double x1 = cos(angle)*MAX_LIDAR_RANGE + x0;
		double y1 = sin(angle)*MAX_LIDAR_RANGE + y0;
		// raytrace
		mt.ranges[i] = raytrace(x0, y0, x1, y1, map);
	}
	return mt;
}

// Returns distance to the first obstacle between (x0,y0) and (x1,y1)
// http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
float babs_slam::raytrace(double x0, double y0, double x1, double y1, nav_msgs::OccupancyGrid map) {
	// x and y components of the ray
	double dx = fabs(x1-x0);
	double dy = fabs(y1-y0);

	// cell we are currently in
	int x = int(floor(x0));
	int y = int(floor(y0));

	// how far we are, units = % of ray length, 0<=t<=1
	double t = 0;

	// how much does t change to move 1 cell horizontally or vertically
	double dt_dx = 1.0 / dx;
	double dt_dy = 1.0 / dy;

	// n - how many cells do we need to traverse
	int n = 1;
	// how much we change x,y for each cell (direction where we are moving)
	int x_inc, y_inc;
	// value of t for next intersection
	double t_next_vertical, t_next_horizontal;

	// initialization

	if (dx == 0)
	{
		// moving vertically
		x_inc = 0;
		t_next_horizontal = dt_dx; // infinity
	}
	else if (x1 > x0)
	{
		// moving to the right
		x_inc = 1;
		n += int(floor(x1)) - x;
		t_next_horizontal = (floor(x0) + 1 - x0) * dt_dx;
	}
	else
	{
		// moving to the left
		x_inc = -1;
		n += x - int(floor(x1));
		t_next_horizontal = (x0 - floor(x0)) * dt_dx;
	}

	if (dy == 0)
	{
		// moving horizontally
		y_inc = 0;
		t_next_vertical = dt_dy; // infinity
	}
	else if (y1 > y0)
	{
		// moving up
		y_inc = 1;
		n += int(floor(y1)) - y;
		t_next_vertical = (floor(y0) + 1 - y0) * dt_dy;
	}
	else
	{
		// moving down
		y_inc = -1;
		n += y - int(floor(y1));
		t_next_vertical = (y0 - floor(y0)) * dt_dy;
	}

	// traverse
	for (; n > 0; --n)
	{
		// Terminate if reached the end of the map, ray, or obstacle
		if (!within_map_bounds(x,y) || map_get_value(map,x,y)>MAP_OCC_THRESH) {
			float distance_in_cells=t*sqrt(pow(x1-x0,2)+pow(y1-y0,2));
			return distance_in_cells*MAP_RESOLUTION;
		}
		if (n==1) {
			// reached end of ray
			float distance_in_cells=sqrt(pow(x1-x0,2)+pow(y1-y0,2));
			return distance_in_cells*MAP_RESOLUTION;
		}

		// Go to the next cell
		if (t_next_vertical < t_next_horizontal)
		{
			y += y_inc;
			t = t_next_vertical;
			t_next_vertical += dt_dy;
		}
		else
		{
			x += x_inc;
			t = t_next_horizontal;
			t_next_horizontal += dt_dx;
		}
	}
}