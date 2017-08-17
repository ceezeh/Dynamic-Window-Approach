#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <unistd.h>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <numeric>
#include <costmap/helper.h>
#include "dwa/dwa.h"
#include "costmap/escapevelocity.h"
#include <chrono>

#define DEBUG
//#define USER_CMD_BIT 2
#define MAP_BIT 1
#define ODOM_BIT 0
#define lo_occ_thres 50
using namespace std;
using namespace mapcontainer;
using namespace std::chrono;

bool isSim = false;

DWA::DWA(const char * topic_t, ros::NodeHandle &n_t) :
		topic(topic_t), n(n_t) {
	this->goalPose = Pose(10, 0);
	// Trajectories.
	for (float i = -M_PI; i < M_PI; i += ANG_STEP) { // Split into 12 angles  for each half.
		trajectories.push_back(i);
		distFromObstacle.push_back(Distance(NULLDIST, NULLDIST, NULLDIST));
	}
	string ns = string(topic_t);
	string isSimStr = ns + "/issim";
	this->n.getParam(isSimStr.c_str(), isSim);
	cout << "Is Sim?: :" << isSim << endl;

	string res = ns + "/resolution";
	float resolution;
	this->n.getParam(res.c_str(), resolution);
	cout << "Resolution:" << resolution << endl;

	float mapsize;
	string swl = ns + "/globalmapwidth";
	this->n.getParam(swl.c_str(), mapsize);
	cout << "Global mpa width: " << mapsize << endl;

	/*
	 * These parameters are used for accessing the right back side of the wheelchair
	 * as the start position to fill or check occupancy.
	 */

	string dtStr = ns + "/dt";
	this->n.getParam(dtStr.c_str(), dt);

	//	 TODO: Verify these parameters.
	horizon = 20; // 10 seconds
	refresh_period = 0; // 1 / dt;

	string acc_lim_v_str = ns + "/acc_lim_v";
	this->n.getParam(acc_lim_v_str.c_str(), acc_lim_v);

	string acc_lim_w_str = ns + "/acc_lim_w";
	this->n.getParam(acc_lim_w_str.c_str(), acc_lim_w);

	string decc_lim_v_str = ns + "/decc_lim_v";
	this->n.getParam(decc_lim_v_str.c_str(), decc_lim_v);

	string decc_lim_w_str = ns + "/decc_lim_w";
	this->n.getParam(decc_lim_w_str.c_str(), decc_lim_w);

	// WC kinematics

	if (isSim) {
		max_trans_vel = .4;
		min_trans_vel = -.4;
	} else {
		max_trans_vel = MAX_LIN_VEL;
		min_trans_vel = -MIN_LIN_VEL;
	}
	max_rot_vel = MAX_ANG_VEL;
	min_rot_vel = -MIN_ANG_VEL;
	// WC dimensions.

	vstep = (max_trans_vel - min_trans_vel) / 3;
	wstep = (max_rot_vel - min_rot_vel) / 3;

	odom = Speed(0, 0);
	dwa_map = MapContainerPtr(
			new MapContainer(resolution, mapsize, WCDimensions(&n, topic_t)));

	deOscillator = DeOscillator();
	deOscillator.changeDir(Pose(0, 0), goalPose);
	// ROS
	DATA_COMPLETE = 3;
	wcDimensions = WCDimensionsPtr(new WCDimensions(&n, topic_t));
	string cmd_topic_name = ns + "/cmd_topic";
	string cmd_topic;
	this->n.getParam(cmd_topic_name.c_str(), cmd_topic);
	cout << "cmd_topic: " << cmd_topic << endl;

	command_pub = n.advertise<geometry_msgs::Twist>(cmd_topic.c_str(), 1);

	firstOdom = true;
	velocity_sub = n.subscribe("odom", 1, &DWA::velocityCallback, this);

	occupancy_sub = n.subscribe("global_map", 1, &DWA::occupancyCallback, this);

	goalPose_sub = n.subscribe("goalpose", 1, &DWA::goalPoseCallback, this);

	dataflag = 0;

}

//DWA::~DWA() {
//	delete this->dwa_map;
//}

void DWA::velocityCallback(const nav_msgs::Odometry& cmd) {
	// Update dataflag.
	dataflag |= (1 << ODOM_BIT);
	this->odom.v = cmd.twist.twist.linear.x;
	this->odom.w = cmd.twist.twist.angular.z; // scaling factor that maps user's command to real world units.
}

void DWA::occupancyCallback(const nav_msgs::OccupancyGrid& og) {
	// Update dataflag.
	dataflag |= (1 << MAP_BIT);
	this->dwa_map->updateMap(og);

	nav_msgs::Odometry odomt;
	odomt.pose.pose = this->dwa_map->getMap().info.origin;
	odomt.twist.twist.angular.z = odom.w;
	odomt.twist.twist.linear.x = odom.v;
	deOscillator.updateOdom(odomt);
}

void DWA::goalPoseCallback(const geometry_msgs::Pose& p) {
	Pose goal = Pose(p.position.x, p.position.y, getYaw(p.orientation));
	updateGoalPose(goal);
}
/*
 * Heading is defined in the first paper on DWA
 * as the bearing of the robot���������s direction from the goal,
 * such that heading is maximum when the robot is facing the goal.
 * It is a measure to motivate the robot to progress towards the goal.
 * It is computed at the position the robot will be in after maximum deceleration from the next time step.
 *
 * We normalise this value to the -1,1 range.
 */
float DWA::computeHeading(Speed candidateSpeed, Pose goal) {
	// Compute goal from user command.
	if (candidateSpeed == Speed(0, 0)) {
		return 0;
	}

	Pose currentPose;
	this->getCurrentPose(currentPose);
	float velHeading = atan2(candidateSpeed.w, candidateSpeed.v);
	float headingToGoal = angDiff(goal.th, currentPose.th);
	bool front = this->deOscillator.isFront();
	float heading;
	if (front) {
		heading = M_PI - angDiff(headingToGoal, velHeading);
	} else {
		heading = -angDiff(headingToGoal, velHeading);
	}
	/*
	 * normalise the test speed as well.
	 * Here we are asking what speed would closely match the user's
	 * intention given his joystick angle of deflection.
	 * We assume that the user is aiming for an instantenous speed
	 * rather than that speed over a long distance.
	 * Thus there is no need to predict intention as a position.
	 * Rather it is sufficient to consider prediction over intended
	 * instantaneous direction and magnitude of motion (i.e over velocity space)
	 */

//	float goalth = goal.w*horizon*dt; // gets the starting motion direction from the user's input.
//	Speed test = candidateSpeed;
//	float candididateth =test.w*horizon*dt;
//
//	// Compute heading here.
//	float heading = M_PI - angDiff(candididateth, goalth);
	// Normalise heading to [0,1]
	heading = wraparound(heading);
	heading = fabs(heading) / M_PI;
	return heading;
}

/*
 * Clearance measures the distance to the closest obstacle on the trajectory.
 * TODO: Populate values on a lookup table to prevent re-evaluation.
 * Clearance is now dynamic.
 */

float DWA::computeClearance(Speed candidateSpeed) {
	// clearance is normalised to [0,1] by d
	if (candidateSpeed == Speed(0, 0)) {
		return 1;
	}
//	int dummy;
	float traj = atan2(candidateSpeed.w, candidateSpeed.v);
	int ind = getClearanceIndex(traj);
	if (ind == trajectories.size())
		ind = 0;
	Distance dist = computeDistToNearestObstacle(candidateSpeed, ind);
	cout << "Traj= " << traj << ", Index= " << ind << endl;
#ifdef DEBUG
	ROS_INFO("Pre clearance: clearance %f", dist.clearance);
#endif

	// Since clearance is the last time dist is used, we reset it here.
	float clearance = (dist.clearance < SAFEZONE) ? 0 : dist.clearance / 2.55;// / 2.550;
//	float ang = (dist.ang<0.05) ? 0 : dist.ang *0.159; // 20 degree safetym
	if (clearance > 1)
		clearance = 1;
	return clearance; /*As done in Pablo et al*/

}

Distance DWA::computeDistToNearestObstacle(Speed candidateSpeed, int i) {

	if (distFromObstacle[i].clearance != NULLDIST) {
		return distFromObstacle[i];
	} else {
		distFromObstacle[i] = computeDistToNearestObstacle(candidateSpeed);
		return distFromObstacle[i];
	}
}
Distance DWA::computeDistToNearestObstacle(Speed candidateSpeed) {
	// Compute rectangular dimension depicting wheelchair in  occupancy map.
	// Compute and normalize clearance.

//	if (equals(candidateSpeed.w, 0)) {
//		for (int i = 0; i < this->dwa_map->getMap().data.size(); i++) {
//			if (this->dwa_map->getMap().data[i] > lo_occ_thres) {
//				int x, y;
//				this->dwa_map->getOccXY(i, x, y);
//				cout << "Searching....obstacle found at x=" << x << ",y=" << y
//						<< endl;
//			}
//		}
//	}
	bool foundObst = false;
	float traj = atan2(candidateSpeed.w, candidateSpeed.v);
	float x, y, th;
	x = y = th = 0;
	float v = copysign(0.1, candidateSpeed.v);
	float w = v * tan(traj);
	if (equals(fabs(traj), M_PI / 2)) {
		w = copysign(0.2, traj);
		v = 0;
	}

	float clearance = 2.55; // 2.55m is maximum detectable distance by sonar
	int count = refresh_period; // we want 2 seconds = 2/dt counts
	float acc = 0;
	for (int i = 0; i < 1; i++) {
		float dx = odom.v * cos(th) * dt;
		float dy = odom.v * sin(th) * dt;
		float ds = vectorNorm(Pose(dx, dy));
//		acc += ds;
		x += dx;
		y += dy;
		th += odom.w * dt;
		th = wraparound(th);
	}

	for (int i = 0; i < horizon; i++) {
		float dx, dy, ds, dth;
		if (i > 0) {
			dx = v * cos(th) * dt;
			dy = v * sin(th) * dt;
			ds = vectorNorm(Pose(dx, dy));
			acc += ds;
			dth = w * dt;
		} else {
			dth = candidateSpeed.w * dt;
			dx = candidateSpeed.v * cos(th + dth) * dt;
			dy = candidateSpeed.v * sin(th + dth) * dt;
			ds = vectorNorm(Pose(dx, dy));
		}
		x += dx;
		y += dy;
		th += dth;
		th = wraparound(th);
//		cout<<"[Compute Clearance] x="<<x<<",y="<<y<<",th="<<th<<", i="<<i<<endl;
//		timestep = i;
		if ((fabs(x / dwa_map->getResolution()) > dwa_map->getNoOfGrids() / 2)
				|| (fabs(y / dwa_map->getResolution())
						> dwa_map->getNoOfGrids() / 2)) {
			cout << "Horizon at edge of localmap!!!" << endl;
			break;
		}
		// the below is expensive to compute so
		// compute only after every 2 seconds into the future.

		Pose pose = Pose(x, y, th);
		if (i > -1) { //For laser sensor.
			bool isObstacle = onObstacle(pose, candidateSpeed,
					(i == 0) ? true : false);

			if (isObstacle) {
				foundObst = true;
				break;
			}
		}

	}
	if (foundObst) {
		float dx = candidateSpeed.v * cos(th) * dt;
		float dy = candidateSpeed.v * sin(th) * dt;
		float ds = vectorNorm(Pose(dx, dy));
		th = fabs(th);
		if (equals(traj, 0) || equals(traj, M_PI))
			return Distance(acc, acc + ds, 2 * M_PI);
		if (equals(fabs(traj), M_PI / 2)) {
			if (equals(candidateSpeed.w, 0))
				cout << "@@Using ang dist..." << endl;
			return Distance(th * 0.55, th * 0.55 + ds, th); // Using Centre to back metric...
		} else
			return Distance(acc, acc + ds, th);
	} else {
		return Distance(clearance, clearance, 2 * M_PI);
	}
}

/*
 * Checks if there is any obstacle in the occupancy grid to obstruct the
 * wheelchair if its centre where at pose. Returns the location of the obstacles.
 */
bool DWA::onObstacle(Pose pose, Speed candidateSpeed, bool escape = 0) {

	//Temp: List all obstacles

//	cout << "Local Wheelchair Pose: [x= " << pose.x << ", y=" << pose.x
//			<< ", th =" << pose.th << "]" << endl;
//	geometry_msgs::Pose pose = req.pose;
	Pose currentPose;
	getCurrentPose(currentPose);
	RealPoint p = RealPoint(pose.x, pose.y);
	rotateFromBody(currentPose, &p);
	pose.x = p.x;
	pose.y = p.y;
	pose.th = angAdd(pose.th, currentPose.th);

	EscapeVelocity escapeVelocity = EscapeVelocity(*(this->wcDimensions.get()));
	for (int k = 0; k < 1; k++) {

		RealPoint topLeft = wcDimensions->getTopLeftCorner1();
		RealPoint topRight = wcDimensions->getTopRightCorner1();
		RealPoint bottomRight = wcDimensions->getBottomRightCorner1();
		RealPoint bottomLeft = wcDimensions->getBottomLeftCorner1();

		RealPoint topLeft2 = wcDimensions->getTopLeftCorner2();
		RealPoint topRight2 = wcDimensions->getTopRightCorner2();
		RealPoint bottomRight2 = wcDimensions->getBottomRightCorner2();
		RealPoint bottomLeft2 = wcDimensions->getBottomLeftCorner2();

		float delta = k * dwa_map->getResolution();
		topLeft += RealPoint(delta, -delta);
		topRight += RealPoint(-delta, -delta);
		bottomRight += RealPoint(-delta, delta);
		bottomLeft += RealPoint(delta, delta);

		topLeft2 += RealPoint(-delta, -delta);
		topRight2 += RealPoint(-delta, -delta);
		bottomRight2 += RealPoint(-delta, delta);
		bottomLeft2 += RealPoint(-delta, delta);
		// Transform corners into pose coordinate.
		rotateFromBody(pose, &topLeft);
		rotateFromBody(pose, &topRight);
		rotateFromBody(pose, &bottomLeft);
		rotateFromBody(pose, &bottomRight);

		rotateFromBody(pose, &topLeft2);
		rotateFromBody(pose, &topRight2);
		rotateFromBody(pose, &bottomLeft2);
		rotateFromBody(pose, &bottomRight2);

		IntPoint topLeftInt;
		dwa_map->realToMap(topLeft, topLeftInt);
		IntPoint topRightInt;
		dwa_map->realToMap(topRight, topRightInt);
		IntPoint bottomLeftInt;
		dwa_map->realToMap(bottomLeft, bottomLeftInt);
		IntPoint bottomRightInt;
		dwa_map->realToMap(bottomRight, bottomRightInt);
		IntPoint topLeftInt2;
		dwa_map->realToMap(topLeft2, topLeftInt2);
		IntPoint topRightInt2;
		dwa_map->realToMap(topRight2, topRightInt2);
		IntPoint bottomLeftInt2;
		dwa_map->realToMap(bottomLeft2, bottomLeftInt2);
		IntPoint bottomRightInt2;
		dwa_map->realToMap(bottomRight2, bottomRightInt2);

		// Now get the map equivalent of points.

		// Compute the outline of the rectangle.

		vector<IntPoint> outline;
		bresenham(topLeftInt.x, topLeftInt.y, topRightInt.x, topRightInt.y,
				outline);

		bresenham(topRightInt.x, topRightInt.y, topLeftInt2.x, topLeftInt2.y,
				outline);
		bresenham(topLeftInt2.x, topLeftInt2.y, topRightInt2.x, topRightInt2.y,
				outline);
		bresenham(topRightInt2.x, topRightInt2.y, bottomRightInt2.x,
				bottomRightInt2.y, outline);
		bresenham(bottomRightInt2.x, bottomRightInt2.y, bottomLeftInt2.x,
				bottomLeftInt2.y, outline);

		bresenham(bottomLeftInt2.x, bottomLeftInt2.y, bottomRightInt.x,
				bottomRightInt.y, outline);
		bresenham(bottomRightInt.x, bottomRightInt.y, bottomLeftInt.x,
				bottomLeftInt.y, outline);
		bresenham(bottomLeftInt.x, bottomLeftInt.y, topLeftInt.x, topLeftInt.y,
				outline);
//
//		cout << "The footplate coordinates: Real[Lx,Ly]=[" << topRight2.x << ","
//				<< topRight2.y << "], Int[Lx,Ly]=[" << topRightInt2.x << ","
//				<< topRightInt2.y << "]Real[Rx,Ry]=[" << bottomRight2.x << ","
//				<< bottomRight2.y << "], Int[Rx,Ry]=[" << bottomRightInt2.x << ","
//				<< bottomRightInt2.y << "]" << endl;
		for (int i = 0; i < outline.size(); i++) {
			IntPoint point = outline[i];
			//			cout << "Probbing Grid.. X:"<<point.x<<", Y: "<<point.y<<endl;
			if (point.x < 0 || point.x > dwa_map->getNoOfGrids() || point.y < 0
					|| point.y > dwa_map->getNoOfGrids()) {
				continue;
			}
			if (this->dwa_map->at(point.x, point.y) > lo_occ_thres) {
				RealPoint realPoint;
				dwa_map->mapToReal(point, &realPoint);
//				cout
//						<< "Probbing...Obstacle found at wheelchair's edge : Realpoint[x="
//						<< realPoint.x << ", y=" << realPoint.y
//						<< "] Intpoint[x=" << point.x << ", y=" << point.y
//						<< "]" << endl << "Current Wheelchair Pose: [x= "
//						<< currentPose.x << ", y=" << currentPose.x << ", th ="
//						<< currentPose.th << "]" << endl;
////						<<"Wheelchair Pose at Collision: [x= "
////						<< pose.x << ", y="
////						<< pose.y << ", th ="
////						<< pose.th << "]"<<endl;

				//Point in local frame:
				this->dwa_map->globalToBody(currentPose, &realPoint);
//				cout << "Obstacle in body frame : Realpoint[x=" << realPoint.x
//						<< ", y=" << realPoint.y << "]" << endl;
				// find point in body frame;
				//Check if should stop now
				if (!escapeVelocity.isInSafetyZone(realPoint)) {
					return true;//This seems strange but it is likely correct.
				}
				escapeVelocity.updateZones(realPoint);

			}
		}

	}
	bool isEscapeVelocity = escapeVelocity.isEscapeVelocity(candidateSpeed);

	if (!isEscapeVelocity) {
		return true;
	} else {
		return false;
	}
}

/*
 *  This function normalises s the forward velocity of the robot and supports fast motion.
 *  We want to include angular velocity as well so as to obey the user's commands whilst supporting faster motion.
 *
 */
float DWA::computeVelocity(Speed candidateSpeed) {
// Normalise velocity to [0, 1] range.
	Speed speed = normaliseSpeed(candidateSpeed);
	return fabs(speed.v);
}
/*
 * Function gets all possible admissible velocities according to the paper.
 * The returned velocities are indexed to align with the trajectory vector variable.
 *
 * This function really establishes an upperbound on velocity since all very slow speeds are theoretically reachable.
 * We optimize this function by only considering trajectories a certain degree of user input.
 */
concurrent_vector<Speed> DWA::getAdmissibleVelocities(
		concurrent_vector<Speed> admissibles, float upperbound = M_PI + 1,
		float lowerbound = -M_PI - 1) {
	admissibles.clear();
// We search for velocities for by keeping a list of radii of curvature.
#pragma omp parallel for
	for (int i = 0; i < trajectories.size(); i++) {
		admissibles.emplace_back(0, 0);
	}
#pragma omp parallel for
	for (int i = 0; i < trajectories.size(); i++) {
		// Construct a speed object from angle and find the clearance.

		if (!isAngleInRegion(trajectories[i], upperbound, lowerbound)) {
			admissibles[i] = Speed(0, 0); // This is because the list of admissibles must match with the list of trajectories.
			cout << "Emplaced admissible zero for traj: " << trajectories[i]
					<< endl;
			continue;
		}
		Speed trajectory;
		if ((trajectories[i] < M_PI / 2) && ((trajectories[i] >= -M_PI / 2))) { // +ve v space
			trajectory.v = max_trans_vel;
		} else {
			trajectory.v = min_trans_vel;
		}

		// need to convert velocities from normalised to real values.
		trajectory.w = trajectory.v * tan(trajectories[i]);
		trajectory.w =
				(trajectory.w > max_rot_vel) ? max_rot_vel :
				(trajectory.w < min_rot_vel) ? min_rot_vel : trajectory.w;
		//
//		int timestep = -1; // Accounts for latency problems.
		Distance dist = computeDistToNearestObstacle(trajectory, i);
		ROS_INFO(
				"Dist from Obstacle for traj %f : lookahead %f, ang: %f, index:%d",
				trajectories[i], dist.lookahead, dist.ang, i);
//		cout << "@@@@@@@Timestep: " << timestep << endl;
//		float linDist = ((dist.clearance < SAFEZONE) || (timestep < 3)) ? 0 : dist.clearance;
		float linDist = (dist.lookahead < SAFEZONE) ? 0 : dist.lookahead;
		// May need to repeat the above for angular distance.
		float va = copysign(sqrt(fabs(2 * linDist * decc_lim_v)), trajectory.v);
		// Here we are simply getting the velocity restriction for each trajectory.
		// In this case, wa is bound to va so a slight deviation from the paper's wa calculation.
		float wa = copysign(sqrt(fabs(2 * dist.ang * decc_lim_w)),
				trajectory.w);

		admissibles[i] = Speed(va, wa);
	}
	return admissibles;
}

DynamicWindow DWA::computeDynamicWindow(DynamicWindow dw) {
	cout << "odom.v: " << odom.v << ", odom.w: " << odom.w << endl;
	dw.upperbound.v = odom.v + acc_lim_v * dt * 10;
	dw.upperbound.w = odom.w + acc_lim_w * dt * 10;
	dw.lowerbound.v = odom.v + decc_lim_v * dt * 10;
	dw.lowerbound.w = odom.w + decc_lim_w * dt * 10;
	return dw;
}

concurrent_vector<Speed> DWA::getResultantVelocities(
		concurrent_vector<Speed> resultantVelocities,
		float upperbound = M_PI + 1, float lowerbound = -M_PI - 1) {

	MyTimer timer;
	timer.start();
	DynamicWindow dw;
	dw = computeDynamicWindow(dw);
	concurrent_vector<Speed> admissibles;
	admissibles.clear();
	admissibles = getAdmissibleVelocities(admissibles, upperbound, lowerbound);
	timer.stop();
	cout << "[DEBUG] Admissible time:" << timer.getLastDuration() << endl;
	bool front = deOscillator.isFront();
// Check direction of goal is in front or behind.
	bool zeroVisited = false;	// ensures that we add v=w= 0 only once.

#pragma omp parallel for
	for (int i = 0; i < trajectories.size(); i++) {
//		cout << "Begin! ADMISSIBLE traj" << trajectories[i] << " : [v="
//				<< admissibles[i].v << ",w=" << admissibles[i].w << "]" << endl;
//		cout << "DW lowerbound traj: [v=" << dw.lowerbound.v << ",w="
//				<< dw.lowerbound.w << "]" << endl;
//		cout << "DW upperbound traj: [v=" << dw.upperbound.v << ",w="
//				<< dw.upperbound.w << "]" << endl;
		if (front) {
			if (fabs(trajectories[i]) > (M_PI / 2 + .1)) {
//				cout << "Facing front ignoring trajectory " << endl;
				continue;
			}
		} else {
			if (fabs(trajectories[i]) < (M_PI / 2 + .1)) {
//				cout << "Facing back ignoring trajectory " << endl;
				continue;
			}
		}
		if (!zeroVisited) {
			resultantVelocities.emplace_back(0, 0);
//			cout << "Emplacing resultant traj: [v=" << 0 << ",w=" << 0 << endl;
			zeroVisited = true;
		}
//		cout << "Trajectory Heading : " << trajectories[i] << endl;
		if (!isAngleInRegion(trajectories[i], upperbound, lowerbound)) {
//			cout << "Trajectory not in angle range " << endl;
			continue;
		}

//		cout << "Passed trajectory Heading : " << trajectories[i] << endl;
		// for very large tan, the data becomes skewed so just use the dw as boundary
		//Here trajectory is either pi/2 or -PI/2
		if (equals(abs(trajectories[i]), M_PI / 2)) {
			float vel = 0;
			float upperbound_w, lowerbound_w;
			if (equals(trajectories[i], M_PI / 2)) {
				upperbound_w = fmin(dw.upperbound.w, admissibles[i].w);
				lowerbound_w = (dw.lowerbound.w < 0) ? 0 : dw.lowerbound.w;
			} else {
				upperbound_w = (dw.upperbound.w > 0) ? 0 : dw.upperbound.w;
				lowerbound_w = fmax(dw.lowerbound.w, admissibles[i].w);
			}
			upperbound_w = fmin(upperbound_w, max_rot_vel);
			lowerbound_w = fmax(lowerbound_w, min_rot_vel);
			float stepw = (upperbound_w - lowerbound_w) / 6;
			if (stepw == 0)
				continue;
//			cout << "upperbound: " << upperbound << "lowerbound" << lowerbound
//					<< "upperbound_w: " << upperbound_w << "lowerbound_w"
//					<< lowerbound_w << endl;
			for (float w = lowerbound_w; w < upperbound_w; w += 0.98 * stepw) {
				if ((w >= lowerbound_w) && ((w <= upperbound_w))) {
//					cout
//							<< "isAngleInRegion(atan2(w, vel), upperbound, lowerbound)"
//							<< isAngleInRegion(atan2(w, vel), upperbound,
//									lowerbound) << endl;
					if (zeroVisited && equals(vel, 0) && equals(w, 0)) {
						continue;
					} else if (equals(vel, 0) && (equals(w, 0))) {
						zeroVisited = true;
					}
					if (isAngleInRegion(atan2(w, vel), upperbound,
							lowerbound)) {
						resultantVelocities.emplace_back(vel, w);
//						cout << "Emplacing resultant traj: [v=" << vel << ",w="
//								<< w << endl;
					} else {
//						cout << "Skipped traj: [v=" << vel << ",w=" << w
//								<< endl;
					}
				}
			}
			continue;
		}

		// For small tan near zero, w = 0

		if (abs(tan(trajectories[i])) < 0.001) {

			float w = 0;
			float lowerbound_v, upperbound_v;
			if ((trajectories[i] < M_PI / 2)
					&& ((trajectories[i] >= -M_PI / 2))) { // +ve v space
				// So compute lowerbound on trajectories.

				upperbound_v = fmin(dw.upperbound.v, admissibles[i].v);
				lowerbound_v = fmax(0, dw.lowerbound.v);
			} else {
				upperbound_v = fmin(0, dw.upperbound.v);
				lowerbound_v = fmax(dw.lowerbound.v, admissibles[i].v);
			}
			upperbound_v = fmin(upperbound_v, max_trans_vel);
			lowerbound_v = fmax(lowerbound_v, min_trans_vel);
			/*
			 * We are assuming focus within a narrow angle so that 0 and PI can not both be within focus.
			 * Thus if zero is present, pi is not.
			 */
			if (isAngleInRegion(0, upperbound, lowerbound)) { // If 0 is present.
				lowerbound_v = (lowerbound_v < 0) ? 0 : lowerbound_v;
			} else {
				upperbound_v = (upperbound_v > 0.0) ? 0 : upperbound_v;
			}
			float step = (upperbound_v - lowerbound_v) / 3;
			if (step == 0)
				continue;
			for (float vel = lowerbound_v; vel <= upperbound_v;
					vel += 0.98 * step) {
				if (zeroVisited && equals(vel, 0) && equals(w, 0)) {
					continue;
				} else if (equals(vel, 0) && (equals(w, 0))) {
					zeroVisited = true;
				}
				if (isAngleInRegion(atan2(w, vel), upperbound, lowerbound)) {
					resultantVelocities.emplace_back(vel, w);
//					cout << "rResultant traj: [v=" << vel << ",w=" << w << endl;
				} else {
//					cout << "Skipped traj: [v=" << vel << ",w=" << w << endl;
				}
			}
			continue;
		}

		float lowerbound_v, upperbound_v;
		if ((trajectories[i] < M_PI / 2) && ((trajectories[i] >= -M_PI / 2))) { // +ve v space
			// So compute lowerbound on trajectories.

			upperbound_v = fmin(dw.upperbound.v, admissibles[i].v);
			lowerbound_v = fmax(0, dw.lowerbound.v);
		} else {
			upperbound_v = fmin(0, dw.upperbound.v);
			lowerbound_v = fmax(dw.lowerbound.v, admissibles[i].v);
		}
		upperbound_v = fmin(upperbound_v, max_trans_vel);
		lowerbound_v = fmax(lowerbound_v, min_trans_vel);

		float step = (upperbound_v - lowerbound_v) / 3;
		if (step == 0)
			continue;
		for (float vel = lowerbound_v; vel <= upperbound_v;
				vel += 0.98 * step) {

			// For small tan near zero, w = 0
			float w = vel * tan(trajectories[i]);

			float upperbound_w, lowerbound_w;
			if (tan(trajectories[i] < 0)) {
				upperbound_w = dw.upperbound.w;
				lowerbound_w = fmax(dw.lowerbound.w, admissibles[i].w);
			} else {
				upperbound_w = fmin(dw.upperbound.w, admissibles[i].w);
				lowerbound_w = dw.lowerbound.w;
			}
			upperbound_w = fmin(upperbound_w, max_rot_vel);
			lowerbound_w = fmax(lowerbound_w, min_rot_vel);
			if ((w >= lowerbound_w) && ((w <= upperbound_w))) {
				if (zeroVisited && equals(vel, 0) && equals(w, 0)) {
					continue;
				} else if (equals(vel, 0) && (equals(w, 0))) {
					zeroVisited = true;
				}
				if (isAngleInRegion(atan2(w, vel), upperbound, lowerbound)) {
					resultantVelocities.emplace_back(vel, w);
//					cout << "rResultant traj: [v=" << vel << ",w=" << w << endl;
				} else {
//					cout << "Skipped traj: [v=" << vel << ",w=" << w << endl;
				}
			}

		}

	}

//	ROS_INFO("Printing resultant velocities ...\n");
//	for (int i = 0; i < resultantVelocities.size(); i++) {
//		ROS_INFO("Vel[%d]; [v = %f, w= %f]", i, resultantVelocities[i].v,
//				resultantVelocities[i].w);
//	}
//	ROS_INFO("Printing resultant velocities ended\n");
//	cout << "END" << endl;
	return resultantVelocities;
}

//void DWA::restrictVelocitySpace(float &upperbound, float &lowerbound,
//		Speed input) {
//	float upperboundt, lowerboundt;
//	deOscillator.getAdmissibleDirection(upperbound, lowerbound);
//	float ang = atan2(input.w, input.v);
//
//	upperboundt = ang + M_PI * 60 / 180;
//	upperboundt = wraparound(upperboundt);
//	lowerboundt = ang - M_PI * 60 / 180;
//	lowerboundt = wraparound(lowerboundt);
//
//	float upper, lower;
////	upper = upperboundt;
////	lowerhttp://marketplace.eclipse.org/marketplace-client-intro?mpc_install=2963451 = lowerboundt;
////
//	if (isAngleInRegion(upperboundt, upperbound, lowerbound)) {
//		upper = upperboundt;
//	} else {
//		upper = upperbound;
//	}http://marketplace.eclipse.org/marketplace-client-intro?mpc_install=2963451http://marketplace.eclipse.org/marketplace-client-intro?mpc_install=2963451http://marketplace.eclipse.org/marketplace-client-intro?mpc_install=2963451http://marketplace.eclipse.org/marketplace-client-intro?mpc_install=2963451
//	if (isAngleInRegion(lowerboundt, upperbound, lowerbound)) {
//		lower = lowerboundt;
//	} else {
//		lower = lowerbound;
//	}
//	upperbound = upper;
//	lowerbound = lower;
//}

/*
 * This is the part that does the probabilistic conditioning based on the user's input.
 */

/*
 * Let G (v,w) = Fn( ������ ������ heading + ������������clearance + ������������velocity), where Fn is arbitrary.
 * C = G(v,w) ��������� exp(-1/(2*s)*(f-h)(f-h)') ���������  W(H), for each of our possible goal location.
 */
std::mutex mylock;
Speed DWA::computeNextVelocity(Speed chosenSpeed) {

	concurrent_vector<Speed> resultantVelocities;
	resultantVelocities.clear();

	/*
	 * Create optimized search space.
	 */

	// Convert to body angle.
//	float dir = -this->dwa_map->getMap().info.origin.position.z;
//	float v = 0.5;
//	Speed goal = Speed(v,v * tan(dir));
	float upperbound = -M_PI - 1;
	float lowerbound = M_PI + 1;

//	restrictVelocitySpace(upperbound, lowerbound, goal);
	deOscillator.getAdmissibleDirection(upperbound, lowerbound);
	ROS_INFO("upperbound: %f, lowerbound: %f", upperbound, lowerbound);
	resultantVelocities = getResultantVelocities(resultantVelocities,
			upperbound, lowerbound);
	float maxCost = 0;
// Put weightings here.
	float alpha = .000001;		// For heading.
	float beta = .2;		// For clearance.
	float gamma = 1;		// For velocity.
	float final_clearance = 0;
	cout << "Number of resultant velocities" << resultantVelocities.size()
			<< endl;

#pragma omp parallel for
	for (int i = 0; i < resultantVelocities.size(); i++) {

		Speed realspeed = resultantVelocities[i];
		float heading = computeHeading(realspeed, goalPose);
		float clearance = computeClearance(realspeed);
// Patch. Clearance of 0 should not be possible here. if ()
		float velocity = computeVelocity(realspeed);
		float G = alpha * heading + beta * clearance + gamma * velocity;

		float cost = G;

#ifdef DEBUG

		ROS_INFO("Printing out DWA parameters for specific velocity ...");
		ROS_INFO("RealVel[v = %f, w= %f], heading=%f, clearance=%f, "
				"velocity = %f, cost = %f, Pose (x: %f, y: %f, th: %f)",
				realspeed.v, realspeed.w, heading, clearance, velocity, cost,
				this->dwa_map->getMap().info.origin.position.x,
				this->dwa_map->getMap().info.origin.position.y,
				getYaw(this->dwa_map->getMap().info.origin.orientation));
#endif
		mylock.lock();
		if (cost > maxCost) {
			maxCost = cost;
			chosenSpeed = realspeed;
			final_clearance = clearance;
		}
		mylock.unlock();
	}
	chosenSpeed = (equals(final_clearance, 0)) ? Speed(0, 0) : chosenSpeed;
	ROS_INFO("Chosen speed: [v=%f, w=%f]", chosenSpeed.v, chosenSpeed.w);
	return chosenSpeed;
}

void DWA::getData() {
	while (dataflag != this->DATA_COMPLETE && ros::ok()) { // Data bits are arranged n order of testing priority.
		ros::spinOnce();

	}
	dataflag = 0;

}

void DWA::updateGoalPose(Pose goal, float dir) {
	if (!(goal == goalPose)) {
//		cout << "NEW Goal" << endl;
		Pose currentPose = Pose(this->dwa_map->getMap().info.origin.position.x,
				this->dwa_map->getMap().info.origin.position.y,
				getYaw(this->dwa_map->getMap().info.origin.orientation));
		this->deOscillator.changeDir(currentPose, goal, dir);
		goalPose = goal;
	}
}

void DWA::run() {
	ros::Rate loop_rate(1 / dt);

	Speed chosenSpeed;
	int maxduration;
	int aveduration;
	MyTimer timer = MyTimer();

	while (ros::ok()) {
		timer.start();
		getData();

		chosenSpeed = this->computeNextVelocity(chosenSpeed);
		timer.stop();

		geometry_msgs::Twist motorcmd;
		motorcmd.linear.x = chosenSpeed.v;
		if (isSim) {
			motorcmd.angular.z = -chosenSpeed.w;
		} else {
			motorcmd.angular.z = chosenSpeed.w;
		}
		command_pub.publish(motorcmd);

		for (int i = 0; i < distFromObstacle.size(); i++) {
			distFromObstacle[i] = Distance(NULLDIST, NULLDIST, NULLDIST);
		}

		loop_rate.sleep();

		ROS_INFO("DWA Max Duration: %d", timer.getMaxDuration());
		ROS_INFO("DWA Average Duration: %d ", timer.getAveDuration());
		ROS_INFO("DWA Last Duration: %d ", timer.getLastDuration());
	}

}
