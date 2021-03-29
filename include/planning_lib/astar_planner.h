#ifndef __ASTAR_PLANNER_H__
#define __ASTAR_PLANNER_H__

#include "planning_lib/base_global_planner.h"
#include <limits>
#include <queue>
#include <algorithm>
#include <iostream>

class AstarPlanner2d: public BaseGlobalPlanner2d
{
public:
	struct AstarNode
	{
		Point2i parent;
		float cost_to_reach;
		float heuristic_cost;
		float cost;

		AstarNode(): parent({-1, -1}), cost_to_reach(std::numeric_limits<float>::max()), 
			heuristic_cost(std::numeric_limits<float>::max()), cost(std::numeric_limits<float>::max()) {}
	};

	AstarPlanner2d() {}

	std::vector<Point2f> compute_plan(Point2f& start, Point2f& goal)
	{
		int si=0, sj=0, gi=0, gj=0;
		if(_map.xy_to_ij(start[0], start[1], si, sj) && _map.xy_to_ij(goal[0], goal[1], gi, gj))
		{
			if(_map(si, sj)==0 && _map(gi, gj)==0)
			{
				Point2i start_i({si, sj});
				Point2i goal_i({gi, gj});
				std::vector<Point2i> path = astar(start_i, goal_i);
				std::vector<Point2f> path_f;
				for(auto path_point: path) 
				{
					Point2f path_point_f;
					_map.ij_to_xy(path_point[0], path_point[1], path_point_f[0], path_point_f[1]);
					path_f.push_back(path_point_f);
				}
				return path_f;
			}
		}
		return std::vector<Point2f>();
	}
private:
	typedef std::pair<Point2i, std::pair<float, float>> PQ_Element;
	std::vector<Point2i> astar(Point2i& start, Point2i& goal)
	{
		int width = _map.get_width(), height = _map.get_height();
		std::vector<AstarNode> nodes(width*height, AstarNode());
		auto comp = [](PQ_Element& a, PQ_Element& b){
			if(a.second.first == b.second.first) return a.second.second > b.second.second;
			return a.second.first > b.second.first;
		};
		std::priority_queue<PQ_Element, std::vector<PQ_Element>, decltype(comp)> pq(comp);
		
		int ind = start[0]*width+start[1];
		nodes[ind].cost_to_reach = 0;
		nodes[ind].heuristic_cost = goal.distance_to(start);
		nodes[ind].cost = nodes[ind].cost_to_reach + nodes[ind].heuristic_cost;
		pq.push({start, std::pair<float, float>{nodes[ind].cost, nodes[ind].cost_to_reach}});

		bool goal_reached = false;
		while(!pq.empty())
		{
			PQ_Element cur = pq.top();
			pq.pop();

			if(goal.is_equal_to(cur.first))
			{
				goal_reached = true;
				break;
			}

			int cur_ind = cur.first[0]*width+cur.first[1];
			for(int i=-1; i<=1; ++i)
			{
				for(int j=-1; j<=1; ++j)
				{
					if(i!=0 || j!=0)
					{
						Point2i neighbor({cur.first[0]+i, cur.first[1]+j});
						if(neighbor[0]>=0 && neighbor[0]<height && neighbor[1]>=0 && neighbor[1]<width)
						{
							if(_map(neighbor[0], neighbor[1])==0)
							{
								ind = neighbor[0]*width+neighbor[1];
								float cost_to_reach = nodes[cur_ind].cost_to_reach + neighbor.distance_to(cur.first);
								float heuristic_cost = goal.distance_to(neighbor);
								float cost = cost_to_reach + heuristic_cost;
								if(cost < nodes[ind].cost)
								{
									pq.push({neighbor, std::pair<float, float>{cost, cost_to_reach}});
									nodes[ind].parent = cur.first;
									nodes[ind].cost_to_reach = cost_to_reach;
									nodes[ind].heuristic_cost = heuristic_cost;
									nodes[ind].cost = cost;
								}
							}
						}
					}
				}
			}
		}

		std::vector<Point2i> path;
		if(goal_reached)
		{
			auto path_point = goal;
			path.push_back(path_point);
			ind = path_point[0]*width+path_point[1];
			while(!(nodes[ind].parent[0]==-1 && nodes[ind].parent[1]==-1))
			{
				path_point = nodes[ind].parent;
				path.push_back(path_point);
				ind = path_point[0]*width+path_point[1];
			}
			std::reverse(path.begin(), path.end());
		}
		return path;
	}
};

#endif