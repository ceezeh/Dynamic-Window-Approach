/*
 * SHARED DWA runs in its own thread.
 */

#ifndef DWA_H
#define DWA_H

#define  USE_MATH_DEFINES
//#define 	DEBUG

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <unistd.h>
#include <stdlib.h>
#include <vector>

#include <math.h>
#include <numeric>
#include <costmap/pose.h>
#include "dwa/speed.h"
#include "dwa/helper.h"
#include "dwa/distance.h"
#include "dwa/concurrent_vector.h"
#include <costmap/mapcontainer.h>
#include <string>
#define INVALIDCMD -2
#define SAFEZONE .17 // T17is defines the safezone for DWA within which clearance is always zero.
const float NULLDIST = -1;
//const int NULLSTEP = -1;
using namespace std;
using namespace mapcontainer;
#define ANG_STEP  0.3926990817

class DWA {
public:
	DWA(const char * topic, ros::NodeHandle &n_t);
	void run();

protected:
	ros::NodeHandle n;
	float dt;
	nav_msgs::Odometry odom_all;
	Pose odom_offset;
	bool firstOdom;
	const Pose& getGoalPose() const {
		return goalPose;
	}

	void getCurrentPose(Pose& p) {
		p.x = this->dwa_map->getMap().info.origin.position.x;
		p.y = this->dwa_map->getMap().info.origin.position.y;
		p.th = getYaw(this->dwa_map->getMap().info.origin.orientation);
	}
	int DATA_COMPLETE;
	int dataflag;
	string topic; // Namespace for yaml variables.

private:
	int getClearanceIndex(float ang) {
		return (ang + M_PI ) / ANG_STEP +.2;
	}
	vector<Distance> distFromObstacle;
	// -----------Occupancy Grid Variables-------------------
	/*
	 * The goalstep is a measure used for our distribution to
	 * determine the resolution of our speed distribution measured
	 * in degrees on an Argand chart of w against v.
	 */
	float refresh_period; // rate at which we evaluate prediction as measures in counts representing time in seconds
	MapContainerPtr dwa_map;
	Pose goalPose;
	WCDimensionsPtr wcDimensions;
//	ros::ServiceClient occ_client;
//----------------- Motor Variables ---------------
	Speed odom;

//-------------------ROS-----------------------

	ros::Publisher command_pub;

	ros::Subscriber velocity_sub;
	ros::Subscriber occupancy_sub;
	ros::Subscriber goalPose_sub;

	void occupancyCallback(const nav_msgs::OccupancyGrid& og);
	void velocityCallback(const nav_msgs::Odometry& cmd);
	void goalPoseCallback(const geometry_msgs::Pose& p);
// ----------------------WC Kinematics------------------------
	float acc_lim_v, acc_lim_w;
	float decc_lim_v, decc_lim_w;
	float max_trans_vel, min_trans_vel;
	float max_rot_vel, min_rot_vel;


	float vstep, wstep;
	/*
	 * These parameters are used for accessing the right back side of the wheelchair
	 * as the start position to fill or check occupancy.
	 */

// -------------DWA----------
	/*
	 * Angles made by normalised linear and angular velocities on an
	 * argand chart are stored as trajectory parameters.
	 */
	vector<float> trajectories;
	Distance computeDistToNearestObstacle(Speed traj);
	Distance computeDistToNearestObstacle(Speed traj, int);
//	float computeDistToNearestObstacle(Speed candidateSpeed, int &timestep);
	concurrent_vector<Speed> getAdmissibleVelocities(
			concurrent_vector<Speed> admissibles, float upperbound,
			float lowerbound);
	DynamicWindow computeDynamicWindow(DynamicWindow dw);

// Assuming const time horizon as goal.
protected:
	float horizon; // time steps in the future.
	float computeHeading(Speed candidateSpeed, Pose goal);
	float computeClearance(Speed candidateSpeed);

	bool onObstacle(Pose pose, Speed, bool);
	float computeVelocity(Speed candidateSpeed);
	virtual void getData();
	concurrent_vector<Speed> getResultantVelocities(
			concurrent_vector<Speed> resultantVelocities, float upperbound,
			float lowerbound);
	virtual Speed computeNextVelocity(Speed chosenSpeed);
	DeOscillator deOscillator;
	void restrictVelocitySpace(float &upperbound, float &lowerbound,
			Speed inputcmd);

	void updateGoalPose(Pose goal, float dir = INVALID_DIR);

}
;

#endif
