/*
 * dwa_general.h
 *
 *  Created on: 14 Aug 2017
 *      Author: wheelchair
 */

#ifndef DWA_INCLUDE_DWA_DWA_GENERAL_H_
#define DWA_INCLUDE_DWA_DWA_GENERAL_H_
#ifndef SAFEZONE
#define SAFEZONE .2 // T17is defines the safezone for DWA within which clearance is always zero.
#endif
#include <vector>
#include "dwa/dwa.h"
#include <iostream>
#include <iomanip>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

using namespace std;

class DWAGen: public DWA {
public:
	DWAGen(const char * topic, ros::NodeHandle &n_t);
private:
	vector<nav_msgs::Odometry > dynamic_obstacles;
	Distance computeDistToNearestObstacle(Speed candidateSpeed);
	Distance computeDistToNearestStaticObstacle(Speed candidateSpeed);
	Distance computeDistToNearestDynamicObstacle(Speed candidateSpeed);
	void getDynamicObstacle(const nav_msgs::Odometry odom);
	ros::Subscriber agentSub0, agentSub1, agentSub2,agentSub3,odomSub;
	void odomCallback(const nav_msgs::Odometry&);
	Pose currPose;
};

#endif /* DWA_INCLUDE_DWA_DWA_GENERAL_H_ */
