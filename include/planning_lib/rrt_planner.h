#ifndef __RRT_PLANNER_H__
#define __RRT_PLANNER_H__

#include "planning_lib/base_global_planner.h"
#include "data_structures/kdtree.h"
#include <random>
#include <unordered_map>

/**
* @brief RRT planner for 2d maps
*/
class RRTPlanner2d: public BaseGlobalPlanner2d
{
public:
	/// @brief Default constructor
	RRTPlanner2d() {}

	/**
	* @brief computes path using RRT from start to goal points
	*/
	std::vector<Point2f> compute_plan(Point2f& start, Point2f& goal)
	{
		return rrt(start, goal, 100000, 0.5);
	}
private:
	/// @brief typedef to store nodes and parents for fast retrieval
	typedef std::unordered_map< std::shared_ptr< KDNode<2, float> >, std::shared_ptr< KDNode<2, float> > > NodeParentMap;

	/**
	* @brief rrt method to compute path between start and goal points
	*
	* A KDTree is used to store the nodes for fast retrieval of nearest nodes
	*
	* @param start start point
	* @param goal goal point
	* @param max_iterations maximum iterations for the RRT process
	* @param max_step maximum step to limit step distance between neighboring waypoints
	* @return return vector of points constituting the path
	*/
	std::vector<Point2f> rrt(Point2f& start, Point2f& goal, int max_iterations, double max_step)
	{
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<float> distrib(-30.0, 30.0);

		// construct a 2 dimensional tree and define a map to save parent corresponding parent nodes
		KDTree<2, float> tree;
		NodeParentMap node_map;

		// insert start point to tree
		auto ptr = tree.insert(start);
		node_map.insert({ptr, nullptr});

		int i=0;
		Point2f cur = start;
		while(cur.distance_to(goal)>max_step && i<max_iterations)
		{
			// generate a random point and use the goal point now and then to bias the computed path towards goal
			Point2f rand_point;
			if(i%10==9) rand_point = goal;
			else rand_point = Point2f({distrib(gen), distrib(gen)});

			// get closest point to the random point in the current tree
			std::shared_ptr< KDNode<2, float> > closest_point_ptr = tree.search(rand_point, 0);
			Point2f closest_point = closest_point_ptr->point;
			// get stop point (which is max_step units from the retrieved closest point) on the line joining closest point and the generated random point
			// this step limits the distance between neighboring waypoints in the final path
			Point2f valid_point = get_stop_point(closest_point, rand_point, max_step);
			// use ray tracing to get the valid point on the line that stop before cutting an occupied cell
			valid_point = ray_trace_sampled(closest_point, valid_point, 200);

			if(!valid_point.is_equal_to(closest_point))
			{
				ptr = tree.insert(valid_point);
				node_map.insert({ptr, closest_point_ptr});
			}
			
			// get current point in the tree that is closest to the goal
			cur = tree.search(goal);
			++i;
		}

		// retrieve path from iterating from the goal and closest point to goal in the tree using the node-parent map
		std::vector<Point2f> path;
		ptr = tree.search(goal, 0); 
		cur = ptr->point;
		if(cur.distance_to(goal)<=max_step) path.push_back(goal);
		while(ptr!=nullptr && node_map.find(ptr)!=node_map.end())
		{
			path.push_back(ptr->point);
			ptr = node_map[ptr];
		}
		return path;
	}

	/**
	* @brief helper function to get the stop point on the line joining stat and end points to limit the max step between neighboring waypoints
	*
	* @param start start point
	* @param end end point
	* @param max_step maximum step for the stop point on the line joining start and end points
	* @return returns the stop point
	*/
	Point2f get_stop_point(Point2f& start, Point2f& end, double max_step)
	{
		double dist = start.distance_to(end);
		if(dist<=max_step) return end;
		else
		{
			Point2f diff = end - start;
			return start + Point2f({diff[0]*float(max_step/dist), diff[1]*float(max_step/dist)});
		}
	}

	/**
	* @brief ray tracing method that generates a set of equidistant samples on the line joining start and end points and checks for the points that donot cut an occupied cell in the map
	* 
	* @note Since this method samples the line into equidistant points, there can be cases where the generated points miss the occupied cells completely.
	* A better procedure is needed
	*
	* @param start start point
	* @param end end point
	* @param nsamples number of equidistant samples to be used on the line
	* @return returns the first valid point on the line joining start and end that doesn't cut an occupied cell when traversed from start point 
	*/
	Point2f ray_trace_sampled(Point2f& start, Point2f& end, int nsamples)
	{
		Point2f diff = end-start;
		float d = 1.0/static_cast<float>(nsamples);
		float d1 = 0.0;
		while(d1<=1)
		{
			Point2f valid_point = start + diff*d1;
			int i, j;
			if(_map.xy_to_ij(valid_point[0], valid_point[1], i, j))
			{
				if(_map(i, j)!=0) break;
			}
			else break;
			d1 += d;
		}
		d1 -= d;
		return start + diff*d1;
	}

	/**
	* @brief ray trace procedure using bresenham ray tracing
	*
	* @note This method retreives all the cells on the descritized map that the line passes through and is better approach than the sampled implementation.
	*
	* @todo This call is sometimes causing seg-fault. Debug the implementation
	*
	* @param start start point
	* @param end end point
	* @return returns the first valid point on the line joining start and end that doesn't cut an occupied cell when traversed from start point 
	*/
	Point2f ray_trace(Point2f& start, Point2f& end)
	{
		int height = _map.get_height(), width = _map.get_width();

		Point2i start_i;
		_map.xy_to_ij(start[0], start[1], start_i[0], start_i[1]);
		start_i[0] = std::min(height-1, std::max(0, start_i[0]));
		start_i[1] = std::min(width-1, std::max(0, start_i[1]));

		Point2i end_i;
		_map.xy_to_ij(end[0], end[1], end_i[0], end_i[1]);
		end_i[0] = std::min(height-1, std::max(0, end_i[0]));
		end_i[1] = std::min(width-1, std::max(0, end_i[1]));

		auto ray_points = bresenham_ray_trace(start_i[0], start_i[1], end_i[0], end_i[1]);
		if(!ray_points.empty())
		{
			if(ray_points[0].is_equal_to(start_i))
			{
				int i=0;
				while(i<ray_points.size() && _map(ray_points[i][0], ray_points[i][1])==0)
				{
					++i;
				}
				--i;
				Point2f stop_point;
				_map.ij_to_xy(ray_points[i][0], ray_points[i][1], stop_point[0], stop_point[1]);
				return stop_point;
			}
			else if(ray_points[ray_points.size()-1].is_equal_to(start_i))
			{
				int i=ray_points.size()-1;
				while(i>=0 && _map(ray_points[i][0], ray_points[i][1])==0)
				{
					--i;
				}
				++i;
				Point2f stop_point;
				_map.ij_to_xy(ray_points[i][0], ray_points[i][1], stop_point[0], stop_point[1]);
				return stop_point;
			}
		}
		return start;
	}

	/**
	* @brief Bresenham ray tracing algorithm
	*
	* @param x0 row index of first point
	* @param y0 column index of first point
	* @param x1 row index of second point
	* @param y1 column index of second point
	* @return return vector cell indices that the line betwee first and second points passes through
	*/
	std::vector<Point2i> bresenham_ray_trace(int x0, int y0, int x1, int y1)
	{
		bool steep = (std::abs(y1-y0) > std::abs(x1-x0));
		if(steep)
		{
			std::swap(x0, y0);
			std::swap(x1, y1);
		}
		
		if(x0>x1)
		{
			std::swap(x0, x1);
			std::swap(y0, y1);
		}

		int dx = x1-x0;
		int dy = std::abs(y1-y0);
		std::vector<Point2i> ray_points;
		int ystep = 1;
		if(y0>=y1) ystep = -1;
		int err = 0;
		int x = x0, y = y0;
		for(int i=0; i<=dx; ++i)
		{
			ray_points.push_back(Point2i({x, y}));
			x += 1;
			err += dy;
			if(2*err>=dx)
			{
				y += ystep;
				err -= dx;
			}
		}
		if(steep)
		{
			for(auto& point: ray_points) std::swap(point[0], point[1]);
		}
		
		return ray_points;
	}
};

#endif