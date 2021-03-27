#ifndef __POINT_TYPES_H__
#define __POINT_TYPES_H__

#include <vector>
#include <stdexcept>
#include <cmath> 

/**
* @brief Point template class
*/
template<unsigned int d, class T>
class Point
{
private:
	std::vector<T> x;

public:
	Point(): x(d, 0) {}

	Point(std::vector<T>& v): x(v) {}

	Point(std::vector<T>&& v): x(v) {}

	T& operator[](int i)
	{
		if(i<0 || i>=d) throw std::domain_error("index out of bounds");
		return x[i];
	}

	const T& operator[](int i) const
	{
		if(i<0 || i>=d) throw std::domain_error("index out of bounds");
		return x[i];
	}

	double distance_to(const Point& point) const
	{
		double sq_dist = 0.0;
		for(int i=0; i<d; ++i) sq_dist += (this->x[i] - point[i])*(this->x[i] - point[i]);
		return std::sqrt(sq_dist);
	}

	bool is_equal_to(const Point& point)
	{
		for(int i=0; i<d; ++i) if(this->x[i]!=point[i]) return false;
		return true;
	}

	Point operator+(const Point& point)
	{
		Point res;
		for(int i=0; i<d; ++i) res[i] = (this->x[i] + point[i]);
		return res;
	}

	Point operator-(const Point& point)
	{
		Point res;
		for(int i=0; i<d; ++i) res[i] = (this->x[i] - point[i]);
		return res;
	}

	Point operator-()
	{
		Point res;
		for(int i=0; i<d; ++i) res[i] = -(this->x[i]);
		return res;
	}

	double magnitude()
	{
		return distance_to(Point());
	}

	T dot(const Point& point)
	{
		T res = 0;
		for(int i=0; i<d; ++i) res += (this->x[i] * point[i]);
		return res;
	}

	void operator+=(const Point& point)
	{
		for(int i=0; i<d; ++i) this->x[i] += point[i];
	}
};

typedef Point<2, int> Point2i;
typedef Point<2, float> Point2f;
typedef Point<2, double> Point2d;

typedef Point<3, int> Point3i;
typedef Point<3, float> Point3f;
typedef Point<3, double> Point3d;

#endif