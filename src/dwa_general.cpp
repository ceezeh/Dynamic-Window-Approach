/*
 * dwa_general.cpp
 *
 *  Created on: 14 Aug 2017
 *      Author: wheelchair
 */

#include "dwa/dwa_general.h"

#include <cppopt/newton_raphson.h>
#include <cppopt/gauss_newton.h>
#include <cppopt/gradient_descent.h>
#define AGENT1_BIT 4
#define AGENT0_BIT 3
#define POSE_BIT 2

DWAGen::DWAGen(const char * topic, ros::NodeHandle &n_t) :
		DWA(topic, n_t) {
	dynamic_obstacles.push_back(nav_msgs::Odometry());
	dynamic_obstacles.push_back(nav_msgs::Odometry());
	this->agentSub0 = n.subscribe("agentodom0", 4, &DWAGen::getDynamicObstacle,
			this);
	this->agentSub1 = n.subscribe("agentodom1", 4, &DWAGen::getDynamicObstacle,
				this);
	this->odomSub = n.subscribe("odom", 4, &DWAGen::odomCallback, this);
	this->DATA_COMPLETE = 31;
}

void DWAGen::odomCallback(const nav_msgs::Odometry& odom) {
	dataflag |= (1 << POSE_BIT);
	this->currPose = Pose(odom.pose.pose.position.x, odom.pose.pose.position.y,
			getYaw(odom.pose.pose.orientation));
}

void DWAGen::getDynamicObstacle(nav_msgs::Odometry odom) {
	if (odom.header.frame_id == "agent0") {
		dataflag |= (1 << AGENT0_BIT);
		dynamic_obstacles[0] = nav_msgs::Odometry(odom);
	} else if (odom.header.frame_id == "agent1") {
		dataflag |= (1 << AGENT1_BIT);
		dynamic_obstacles[1] = nav_msgs::Odometry(odom);
	}
}

Distance DWAGen::computeDistToNearestStaticObstacle(Speed candidateSpeed) {
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

Distance DWAGen::computeDistToNearestObstacle(Speed candidateSpeed) {
	Distance d1 = computeDistToNearestDynamicObstacle(candidateSpeed);
	Distance d2 = computeDistToNearestStaticObstacle(candidateSpeed);

	if (d1.clearance < d2.clearance) {
		return d1;
	} else {
		return d2;
	}

}
Distance DWAGen::computeDistToNearestDynamicObstacle(Speed candidateSpeed) {
	float dmin = 2.55;
	float ang = 2 * M_PI;
	for (int i = 0; i < this->dynamic_obstacles.size(); i++) {

		// Convert obstacle position to the wheelchair's body frame.
		nav_msgs::Odometry odom = this->dynamic_obstacles[i];
		RealPoint obst = RealPoint(odom.pose.pose.position.x,
				odom.pose.pose.position.y);

		toBodyFrame(this->currPose, obst);
		// If obstacle is very far away, ignore it.
		if (vectorNorm(obst) > 3)
			break;

		// Get orientation of obstacle.
		float heading = getYaw(odom.pose.pose.orientation);
		heading = angDiff(heading, this->currPose.th); // Heading in body frame.

		const float Cc = odom.twist.twist.linear.x * cos(heading);
		const float Cs = odom.twist.twist.angular.z * sin(heading);

		const float vr = candidateSpeed.v;
		const float wr = candidateSpeed.w;

		const float xob = obst.x;
		const float yob = obst.y;

		const float r = abs(vr / wr);
		float sigt = 1;
		if (equals(copysign(1.0, vr), copysign(1.0, wr))) {
			sigt = -1;
		}
		const float sig = sigt;
		cppopt::F df =
				[&](const cppopt::Matrix &x) -> cppopt::Matrix {
					cppopt::Matrix d(1, 1);

					d(0) =
					(Cs - r*wr*cos(sig*M_PI/2 + x(0)*wr))*((yob) - (r)*(sin(sig*M_PI/2 + (x(0))*(wr)) -sig* 1)
							+ (Cs)*(x(0))) + ((Cc) + sin(sig*M_PI/2 + (x(0))*(wr))*(r)*(wr))*(xob
							+ Cc*x(0) - r*cos(sig*M_PI/2 + x(0)*wr)) + ((Cs) - cos(sig*M_PI/2 + (x(0))*(wr))*(r)*(wr))*(yob
							+ Cs*x(0) - r*(sin(sig*M_PI/2 + x(0)*wr) -sig* 1)) + (Cc + r*wr*sin(sig*M_PI/2 + x(0)*wr))*((xob)
							- cos(sig*M_PI/2 + (x(0))*(wr))*(r) + (Cc)*(x(0)));

					return d;
				};
//		cppopt::F ddf =
//				[&](const cppopt::Matrix &x) -> cppopt::Matrix {
//					cppopt::Matrix d(1, 1);
//
//					d(0) = 2*((Cs) - cos(sig*M_PI/2 + (x(0))*(wr))*(r)*(wr))*(Cs - r*wr*cos(sig*M_PI/2 + x(0)*wr))
//					+ 2*((Cc) + sin(sig*M_PI/2 + (x(0))*(wr))*(r)*(wr))*(Cc + r*wr*sin(sig*M_PI/2 + x(0)*wr))
//					+ r*pow(wr,2)*cos(sig*M_PI/2 + x(0)*wr)*((xob) - cos(sig*M_PI/2 + (x(0))*(wr))*(r) + (Cc)*(x(0)))
//					+ cos(sig*M_PI/2 + (x(0))*(wr))*(r)*pow(wr,2)*(xob + Cc*x(0) - r*cos(sig*M_PI/2 + x(0)*wr))
//					+ r*pow(wr,2)*sin(sig*M_PI/2 + x(0)*wr)*((yob) - (r)*(sin(sig*M_PI/2 + (x(0))*(wr)) -sig)
//							+ (Cs)*(x(0))) + sin(sig*M_PI/2 + (x(0))*(wr))*(r)*pow(wr,2)*(yob + Cs*x(0) -
//							r*(sin(sig*M_PI/2 + x(0)*wr) -sig));
//					return d;
//				};
		cppopt::Matrix x(1, 1);
		x(0) = 0.0;

		// Iterate while norm of the first order derivative is greater than some predefined threshold.
		cppopt::ResultInfo ri = cppopt::SUCCESS;
		while (ri == cppopt::SUCCESS && df(x).norm() > 0.01f) {
			ri = cppopt::gradientDescent(df, x, 0.1);
//			std::cout << std::fixed << std::setw(3) << "Parameters: "
//					<< x.transpose() << " Error: " << df(x).norm() << std::endl;
		}

//		std::cout << "Found a " << (ddf(x)(0) < 0.f ? "Maximum" : "Minimum")
//				<< std::endl;

		float d;
		if (x(0) > 0) {

			float t = x(0);
			d = pow(
					pow(yob - r * (sin(t * wr + sig * M_PI / 2) - sig) + Cs * t,
							2)
							+ pow(
									xob + Cc * t
											- r * cos(t * wr + sig * M_PI / 2),
									2), 0.5);

			// How do we ensure or guarantee non collision?
			if (t < 6 && d < 1.5) {
				d = 0;
				ang = 0;
			}
		} else {
			d = 2.55;
		}

		if (d < dmin) {
			dmin = d;

		}
	}
	cout << "Dmin:" << dmin << endl;
	return Distance(dmin, dmin, ang);

}
