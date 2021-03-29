#ifndef __BASE_GLOBAL_PLANNER_H__
#define __BASE_GLOBAL_PLANNER_H__

#include "planning_lib/occupancy_grid2d.h"

class BaseGlobalPlanner2d
{
public:
	BaseGlobalPlanner2d() {}

	void set_map(std::vector<int>& data, float resolution, int width, int height, Point2f origin)
	{
		_map.set(data, resolution, width, height, origin);
	}

	virtual std::vector<Point2f> compute_plan(Point2f& start, Point2f& goal) = 0;
protected:
	OccupancyGrid2d _map;
};

#endif