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
#include "dwa/pose.h"
#include "dwa/speed.h"
#include "dwa/helper.h"
#include "dwa/map.h"
#include <map/mapcontainer.h>
#include <string>
#define INVALIDCMD -2
#define SAFEZONE .1 // This defines the safezone for DWA within which clearance is always zero.
using namespace std;

class DWA {
public:
	DWA(const char * topic, ros::NodeHandle &n_t);
	void run();



protected:
	ros::NodeHandle n;
	float dt;
	nav_msgs::Odometry odom_all;
	const Pose& getGoalPose() const {
			return goalPose;
		}

	int DATA_COMPLETE;
	int dataflag;
	string topic; // Namespace for yaml variables.
private:

	// -----------Occupancy Grid Variables-------------------
	/*
	 * The goalstep is a measure used for our distribution to
	 * determine the resolution of our speed distribution measured
	 * in degrees on an Argand chart of w againt v.
	 */
	float refresh_period; // rate at which we evaluate prediction as measures in counts representing time in seconds
	MapContainerPtr dwa_map;
	Pose goalPose;
//----------------- Motor Variables ---------------
	Speed odom;

//-------------------ROS-----------------------

	ros::Publisher command_pub;

	ros::Subscriber odom_sub;
	ros::Subscriber occupancy_sub;
	ros::Subscriber goalPose_sub;

	void occupancyCallback(const nav_msgs::OccupancyGrid& og);
	void odomCallback(const nav_msgs::Odometry& cmd);
	void goalPoseCallback(const geometry_msgs::Pose& p);
// ----------------------WC Kinematics------------------------
	float acc_lim_v, acc_lim_w;
	float decc_lim_v, decc_lim_w;
	float max_trans_vel, min_trans_vel;
	float max_rot_vel, min_rot_vel;

	float wc_length, wc_width;
	float vstep, wstep;
	/*
	 * These parameters are used for accessing the right back side of the wheelchair
	 * as the start position to fill or check occupancy.
	 */
	float length_offset, width_offset;
// -------------DWA----------
	/*
	 * Angles made by normalised linear and angular velocities on an
	 * argand chart are stored as trajectory parameters.
	 */
	vector<float> trajectories;
	float computeDistToNearestObstacle(Speed candidateSpeed);
	vector<Speed> getAdmissibleVelocities(vector<Speed> admissibles,
			float upperbound, float lowerbound);
	DynamicWindow computeDynamicWindow(DynamicWindow dw);

// Assuming const time horizon as goal.
protected:
	float horizon; // time steps in the future.
	float computeHeading(Speed candidateSpeed, Pose goal);
	float computeClearance(Speed candidateSpeed);

	bool onObstacle(Pose pose);
	float computeVelocity(Speed candidateSpeed);
	virtual void getData();
	vector<Speed> getResultantVelocities(vector<Speed> resultantVelocities,
			float upperbound, float lowerbound);
	virtual Speed computeNextVelocity(Speed chosenSpeed);
	DeOscillator deOscillator;
	void restrictVelocitySpace(float &upperbound, float &lowerbound,
			Speed inputcmd);

	void updateGoalPose(Pose goal,float dir=INVALID_DIR);

}
;

#endif
