#ifndef MAP_H
#define MAP_H
#include "ros/ros.h"
#include "red_fundamentals/Pose.h"
#include "red_fundamentals/Grid.h"
#include "red_fundamentals/Row.h"
#include "red_fundamentals/Cell.h"

class Map
{
	public:
		red_fundamentals::Grid map;
		int row, column, orientation;
		size_t num_rows, num_columns;
		ros::NodeHandle n;
		Map();
};

#endif

