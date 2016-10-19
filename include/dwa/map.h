#ifndef MAP_H
#define MAP_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <unistd.h>
#include <pthread.h>
#include <sstream>
#include <stdlib.h>
#include <cstdint>
#include <boost/circular_buffer.hpp>
#include <memory>
#include <map/helper.h>
using namespace std;
class DWAMap {
public:
	DWAMap(float resolution_t, float sideWidth) :
			resolution(resolution_t) {
		noOfGrids = int(0.5 + sideWidth / resolution);
		this->map.info.height = this->map.info.width = noOfGrids;
		this->map.info.resolution = resolution;
		map.info.origin.position.x = int(noOfGrids / 2 + 0.5); // ceiling of approximation.
		map.info.origin.position.y = int(noOfGrids / 2 + 0.5); // ceiling of approximation.
	}

	void updateMap(nav_msgs::OccupancyGrid newMap) {
		map.header.stamp = newMap.header.stamp;
		map.info = newMap.info;
		map.data.clear();
		for (int i = 0; i < newMap.data.size(); i++) {
			map.data.push_back(newMap.data[i]);
		}
	}

	int at(int xindx, int yindx) {
		int indx = getIndex(xindx, yindx);
		indx = wraparoundMap(indx);
		return this->map.data[indx];
	}

	int getIndex(int xindx, int yindx) {
		int indx = xindx + noOfGrids * yindx;
		return indx;
	}

// Takes in point in the boday frame.
	void realToMap(RealPoint real, IntPoint& ipoint) {
		int indx, x, y;
		x = this->map.info.origin.position.x + (real.x / resolution);
		y = this->map.info.origin.position.x + (real.y / resolution);
		indx = getIndex(x, y);
		indx = wraparoundMap(indx);
		getOccXY(indx, x, y);
		ipoint.x = x;
		ipoint.y = y;
	}

	void mapToReal(IntPoint mappoint, RealPoint *point) {
		int x = mappoint.x;
		int y = mappoint.y;
		point->x = (x - this->map.info.origin.position.x) * resolution;
		point->y = (y - this->map.info.origin.position.y) * resolution;
	}

private:
	float resolution; // The size in m of each occupancy grid.
	int noOfGrids; // number of grids in one side of the square occupancy.
	nav_msgs::OccupancyGrid map;

	void getOccXY(int indx, int &x, int &y) {
		x = indx % noOfGrids;
		y = (indx - x) / noOfGrids;
	}
	int wraparoundMap(int indx) {
		int x, y;
		getOccXY(indx, x, y);

		if ((x < 0) || (x >= noOfGrids)) {
			ROS_WARN(
					"OVERLAP accessing index. Invalid x: %d. Index out of Map range.",
					x);
			x += (x < 0) ? noOfGrids : -noOfGrids;
		}
		if ((y < 0) || (y >= noOfGrids)) {
			ROS_WARN(
					"OVERLAP accessing index. Invalid y: %d. Index out of Map range.",
					y);
			y += (y < 0) ? noOfGrids : -noOfGrids;
		}
		return indx;
	}

};
typedef std::unique_ptr<DWAMap> DWAMapPtr;
#endif
