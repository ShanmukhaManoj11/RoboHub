#ifndef __POINT_TYPES_H__
#define __POINT_TYPES_H__

#include <vector>
#include <stdexcept>
#include <cmath> 

/**
* @brief Point template class
*
* As an example, a 3 dimensional point with float values can be stored as a `Point<3, float>` object
*/
template<unsigned int d, class T>
class Point
{
private:
	/// @brief `d`-dimensional data a vector of elements of type `T`
	std::vector<T> x;

public:
	/**
	* @brief default constructor
	*/
	Point(): x(d, 0) {}

	/**
	* @overload
	*/
	Point(std::vector<T>& v): x(v) {}

	/**
	* @overload
	*/
	Point(std::vector<T>&& v): x(v) {}

	/**
	* @brief overload [] operator to access element in \f$i^{th}\f$ dimension
	*/
	T& operator[](int i)
	{
		if(i<0 || i>=d) throw std::domain_error("index out of bounds");
		return x[i];
	}

	/**
	* @overload
	*/
	const T& operator[](int i) const
	{
		if(i<0 || i>=d) throw std::domain_error("index out of bounds");
		return x[i];
	}

	/**
	* @brief compute distance from *this* point to a query point
	*
	* @param point query point to find the distance to
	* @return distance to the input point
	*/
	double distance_to(const Point& point) const
	{
		double sq_dist = 0.0;
		for(int i=0; i<d; ++i) sq_dist += (this->x[i] - point[i])*(this->x[i] - point[i]);
		return std::sqrt(sq_dist);
	}

	/**
	* @brief check if query point is equal to *this* point
	*
	* @param point query point
	* @return returns true if input point is exactly equal to *this* point, else returns false
	*/
	bool is_equal_to(const Point& point)
	{
		for(int i=0; i<d; ++i) if(this->x[i]!=point[i]) return false;
		return true;
	}

	/**
	* @brief overload + operator to add *this* point and an input point
	*
	* @param point point to add to *this* point
	* @return returns a new point that is sum of *this* point and the input point
	*/
	Point operator+(const Point& point)
	{
		Point res;
		for(int i=0; i<d; ++i) res[i] = (this->x[i] + point[i]);
		return res;
	}

	/**
	* @brief overload - operator to subtract an input point from *this* point
	*
	* @param point point to subtract from *this* point
	* @return returns a new point that is subtraction of *this* point and the input point
	*/
	Point operator-(const Point& point)
	{
		Point res;
		for(int i=0; i<d; ++i) res[i] = (this->x[i] - point[i]);
		return res;
	}

	/**
	* @brief overload - (unary) operator to negate *this* point
	*
	* @return returns a new point that is negative of *this* point
	*/
	Point operator-()
	{
		Point res;
		for(int i=0; i<d; ++i) res[i] = -(this->x[i]);
		return res;
	}

	/**
	* @brief overload * operator to mulitply a scalar value to elements of *this* point
	*
	* @return returns the scaled point
	*/
	Point operator*(T k)
	{
		Point res;
		for(int i=0; i<d; ++i) res[i] = k*(this->x[i]);
		return res;
	}

	/**
	* @brief compute magnitude of *this* point
	*
	* @return magnitude as double value
	*/
	double magnitude()
	{
		return distance_to(Point());
	}

	/**
	* @brief compute dot product of two points
	*
	* @param point input point to perform dot product with *this* point
	* @return returns scalar result of the dot product
	*/
	T dot(const Point& point)
	{
		T res = 0;
		for(int i=0; i<d; ++i) res += (this->x[i] * point[i]);
		return res;
	}

	/**
	* @brief overload += operator for inplace addition operaton
	*
	* @param point input point to add to *this* point in-place
	*/
	void operator+=(const Point& point)
	{
		for(int i=0; i<d; ++i) this->x[i] += point[i];
	}
};

// typedef commonly used points
typedef Point<2, int> Point2i;
typedef Point<2, float> Point2f;
typedef Point<2, double> Point2d;

typedef Point<3, int> Point3i;
typedef Point<3, float> Point3f;
typedef Point<3, double> Point3d;

#endif