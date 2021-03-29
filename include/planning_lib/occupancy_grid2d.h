#ifndef __OCCUPANCY_GRID2D_H__
#define __OCCUPANCY_GRID2D_H__

#include "data_structures/point_types.h"

/**
* @brief Occupacy grid for 2d maps
*
* @details Defines interface for 2-dimensional occupancy gird maps. Value at each cell in the grid,
* describes the probability of the cell being occupied.
*/
class OccupancyGrid2d
{
public:
	/// @brief default constructor
	OccupancyGrid2d() {}

	/**
	* @brief set occupancy grid data
	*
	* @param data occupancy data of 2d map expressed as 1d array stored in row major order
	* @param resolution float value specifying map resolution
	* @param width width of map in number of cells
	* @param height height of map in number of cells
	* @param origin Point2f origin of map in world coordinates (in meters)
	*/
	void set(std::vector<int>& data, float resolution, int width, int height, Point2f origin)
	{
		_data = data;
		_resolution = resolution;
		_width = width;
		_height = height;
		_origin = origin;
	}

	/**
	* @brief Convert (x,y) to map indices (i,j)
	*
	* @param x input x (in world coordinates)
	* @param y input y (in world coordinates)
	* @param i variable to store computed row index
	* @param j variable to store computed column index
	* @return returns true if the computed (i,j) is within map bounds, else return false
	*/
	bool xy_to_ij(float x, float y, int& i, int& j)
	{
		i = std::floor((x-_origin[0])/_resolution);
		j = std::floor((y-_origin[1])/_resolution);
		return i>=0 && i<_height && j>=0 && j<_width;
	}

	/**
	* @brief Convert map indices (i,j) to world coordinates (x,y)
	*
	* @param i row index of the map
	* @param j column index of the map
	* @param x computed x value in world coordinate
	* @param y computed y value in world coordinates
	*/
	void ij_to_xy(int i, int j, float& x, float& y)
	{
		x = i*_resolution + _origin[0];
		y = j*_resolution + _origin[1];
	}

	/// @brief get map resolution
	float get_resolution() 
	{
		return _resolution;
	}
	
	/// @brief get map width
	int get_width() 
	{
		return _width;
	}

	/// @brief get map height
	int get_height()
	{
		return _height;
	}

	/**
	* @brief overload () operator to access grid cell at index (i,j)
	*
	* @param i row index of the map
	* @param j column index of the map
	* @return reference to the value at cell (i,j) in the map
	*/
	int& operator()(int i, int j)
	{
		return _data[i*_width+j];
	}

	/**
	* @overload
	*/
	int& operator()(int i)
	{
		return _data[i];
	}

private:
	/// @brief 2d occupancy probability data stored in row-major order linearly
	std::vector<int> _data;

	/// @brief map resolution
	float _resolution;

	/// @brief map width as number of map columns
	int _width;

	/// @brief map height as number of map rows
	int _height;

	/// @brief map origin in world coordinates
	Point2f _origin;
};

#endif