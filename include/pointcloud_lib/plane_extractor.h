#ifndef __PLANE_EXTRACTOR_H__
#define __PLANE_EXTRACTOR_H__

#include "data_structures/point_types.h"
#include <unordered_set>
#include <ctime>
#include <cstdlib>

/**
* @brief PlaneExtractor class to extract maximum plane for a point cloud
*
* maximum plane is the plane that passes through maximum number of points in the current point cloud
*/
template<class T>
class PlaneExtractor
{
	// currently only support points with float and double values
	static_assert(std::is_same<T, float>::value || std::is_same<T, double>::value, "only supports float and double points");
public:
	/**
	* @brief Default constructor
	*/
	PlaneExtractor() {}

	/**
	* @brief method to extract plane from pointcloud represented as vector of points
	*
	* @param pointvec vector of points
	* @param max_iterations maximum iterations for RANSAC
	* @param dist_thresh distance threshold from plane to say a point is on the plane
	* @return returns pair of vector of points of form {plane points, other points}
	*/
	std::pair< std::vector< Point<3, T> >, std::vector< Point<3, T> > > extract_plane(std::vector< Point<3, T> >& pointvec, int max_iterations, double dist_thresh)
	{
		// get plane inlier indices using RANSAC
		auto inliers = extract_plane_ransac(pointvec, max_iterations, dist_thresh);
		// differentiate inliers and other points into 2 point vectors
		std::vector< Point<3, T> > plane_points;
		std::vector< Point<3, T> > other_points;
		for(int i=0; i<pointvec.size(); ++i)
		{
			if(inliers.find(i)==inliers.end()) other_points.push_back(pointvec[i]);
			else plane_points.push_back(pointvec[i]);
		}
		return {plane_points, other_points};
	} 
private:
	/**
	* @brief estimate plane passing through 3 points
	*
	* if x0, x1, and x2 are 3 3d-points then coefficients of the plane passing through these points (a, b, c, d) can be calculated as follows,  
	* `d = -(a*x0[0] + b*x0[1] + c*x0[2])`  
	* (a, b, c) represents the plane normal and can be calcualted as cross-product of vectors `x1x0` and `x2x0`
	*/
	Point<4, T> estimate_plane_through(std::vector< Point<3, T> >&& points)
	{
		// vector x1x0 = x1-x0
		Point<3, T> v1 = points[1]-points[0];
		// vector x2x0 = x2-x0
		Point<3, T> v2 = points[2]-points[0];

		Point<4, T> coefs;
		coefs[0] = (v1[1]*v2[2])-(v2[1]*v1[2]);
		coefs[1] = (v2[0]*v1[2])-(v1[0]*v2[2]);
		coefs[2] = (v1[0]*v2[1])-(v2[0]*v1[1]);
		coefs[3] = -(coefs[0]*points[0][0] + coefs[1]*points[0][1] + coefs[2]*points[0][2]);
		return coefs;
	}

	/**
	* @brief extract indices of points that a maximum plane will fit using RANSAC
	*/
	std::unordered_set<int> extract_plane_ransac(std::vector< Point<3, T> >& pointvec, int max_iterations, double dist_thresh)
	{
		std::unordered_set<int> inliers;
		srand(time(NULL));

		int n = pointvec.size();
		for(int i=0; i<max_iterations; ++i)
		{
			std::unordered_set<int> inliers_this_iteration;
			int i0 = rand()%n;
			int i1 = rand()%n;
			while(i0==i1) i1 = rand()%n;
			int i2 = rand()%n;
			while(i0==i2 || i1==i2) i2 = rand()%n;
			auto coefs = estimate_plane_through({pointvec[i0], pointvec[i1], pointvec[i2]});
			for(int j=0; j<n; ++j)
			{
				if(i0==j || i1==j || i2==j) inliers_this_iteration.insert(j);
				else
				{
					double dist = std::abs(coefs[0]*pointvec[j][0] + coefs[1]*pointvec[j][1] + coefs[2]*pointvec[j][2] + coefs[3])/std::sqrt(coefs[0]*coefs[0] + coefs[1]*coefs[1] + coefs[2]*coefs[2]);
					if(dist <= dist_thresh) inliers_this_iteration.insert(j);
				}
			}
			if(inliers.size() < inliers_this_iteration.size()) inliers = inliers_this_iteration;
		}
		return inliers;
	}
};

#endif