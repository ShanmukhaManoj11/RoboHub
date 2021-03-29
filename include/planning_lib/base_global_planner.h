#ifndef __BASE_GLOBAL_PLANNER_H__
#define __BASE_GLOBAL_PLANNER_H__

#include "planning_lib/occupancy_grid2d.h"

/**
* @brief Base global planner using 2d occupancy maps
*
* @details This provides an abstract interface wrapping around an occupancy grid data. 
* Planner interfaces inheriting this base class will define ths `compute_plan` function tht computes
* path from a 2d start point to a goal point
*/
class BaseGlobalPlanner2d
{
public:
	/// @brief default constructor
	BaseGlobalPlanner2d() {}

	/// @brief set occupancy grid data
	void set_map(std::vector<int>& data, float resolution, int width, int height, Point2f origin)
	{
		_map.set(data, resolution, width, height, origin);
	}

	/**
	* @brief virtual function that needs to be defined by derived planner classes
	*
	* This generate a vector of 2d points (waypoints) that constitute a path from start point to a goal point
	*
	* @param start start point
	* @param goal goal point
	* @return returns vector of waypoints from start to goal
	*/
	virtual std::vector<Point2f> compute_plan(Point2f& start, Point2f& goal) = 0;
protected:
	/// @brief occupancy grid describing the world map
	OccupancyGrid2d _map;
};

#endif