#ifndef __POINT_UTILS_H__
#define __POINT_UTILS_H__

#include "data_structures/point_types.h"
#include <eigen3/Eigen/Dense>

/**
* @brief compute mean of points
*/
template<unsigned int d, class T>
Point<d, T> compute_mean(std::vector< Point<d, T> >& pointvec)
{
	Point<d, T> mean;
	for(auto p: pointvec) mean += p;
	for(int i=0; i<d; ++i) mean[i] /= pointvec.size();
	return mean;
}

template<unsigned int d, class T>
Eigen::Matrix<T, d, d> compute_covariance_matrix(std::vector< Point<d, T> >& pointvec)
{
	// covariance mat cov = ATranspose * A, where A = [[x11,x12,...,x1d],[x21,x22,...,x2d],...] - nxd matrix, x1 = point1-mean 
	Eigen::Matrix<T, d, d> cov;
	Point<d, T> mean = compute_mean(pointvec);
	for(int i=0; i<d; ++i)
	{
		for(int j=0; j<=i; ++j)
		{
			cov(i, j) = 0;
			for(auto p: pointvec)
			{
				cov(i, j) += (p[i]-mean[i])*(p[j]-mean[j]);
			}
			cov(i, j) /= pointvec.size();
			cov(j, i) = cov(i, j);
		}
	}
	return cov;
}

#endif