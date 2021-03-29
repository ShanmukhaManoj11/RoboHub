#ifndef __OCCUPANCY_GRID2D_H__
#define __OCCUPANCY_GRID2D_H__

#include "data_structures/point_types.h"

class OccupancyGrid2d
{
public:
	OccupancyGrid2d() {}

	void set(std::vector<int>& data, float resolution, int width, int height, Point2f origin)
	{
		_data = data;
		_resolution = resolution;
		_width = width;
		_height = height;
		_origin = origin;
	}

	bool xy_to_ij(float x, float y, int& i, int& j)
	{
		i = std::floor((x-_origin[0])/_resolution);
		j = std::floor((y-_origin[1])/_resolution);
		return i>=0 && i<_height && j>=0 && j<_width;
	}

	void ij_to_xy(int i, int j, float& x, float& y)
	{
		x = i*_resolution + _origin[0];
		y = j*_resolution + _origin[1];
	}

	float get_resolution() 
	{
		return _resolution;
	}
	
	int get_width() 
	{
		return _width;
	}

	int get_height()
	{
		return _height;
	}

	int& operator()(int i, int j)
	{
		return _data[i*_width+j];
	}

	int& operator()(int i)
	{
		return _data[i];
	}

private:
	std::vector<int> _data;
	float _resolution;
	int _width;
	int _height;
	Point2f _origin;
};

#endif