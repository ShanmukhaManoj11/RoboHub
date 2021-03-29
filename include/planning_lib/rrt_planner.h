#ifndef __RRT_PLANNER_H__
#define __RRT_PLANNER_H__

#include "planning_lib/base_global_planner.h"
#include "data_structures/kdtree.h"
#include <random>
#include <unordered_map>

class RRTPlanner2d: public BaseGlobalPlanner2d
{
public:
	RRTPlanner2d() {}

	std::vector<Point2f> compute_plan(Point2f& start, Point2f& goal)
	{
		return rrt(start, goal, 100000, 0.5);
	}
private:
	typedef std::unordered_map< std::shared_ptr< KDNode<2, float> >, std::shared_ptr< KDNode<2, float> > > NodeParentMap;
	std::vector<Point2f> rrt(Point2f& start, Point2f& goal, int max_iterations, double max_step)
	{
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<float> distrib(-30.0, 30.0);

		KDTree<2, float> tree;
		NodeParentMap node_map;

		auto ptr = tree.insert(start);
		node_map.insert({ptr, nullptr});

		int i=0;
		Point2f cur = start;
		while(cur.distance_to(goal)>max_step && i<max_iterations)
		{
			Point2f rand_point;
			if(i%10==9) rand_point = goal;
			else rand_point = Point2f({distrib(gen), distrib(gen)});

			std::shared_ptr< KDNode<2, float> > closest_point_ptr = tree.search(rand_point, 0);
			Point2f closest_point = closest_point_ptr->point;
			Point2f valid_point = get_stop_point(closest_point, rand_point, max_step);
			valid_point = ray_trace_sampled(closest_point, valid_point, 200);

			if(!valid_point.is_equal_to(closest_point))
			{
				ptr = tree.insert(valid_point);
				node_map.insert({ptr, closest_point_ptr});
			}
			
			cur = tree.search(goal);
			++i;
		}

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