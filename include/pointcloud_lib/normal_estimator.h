#ifndef __NORMAL_ESTIMATOR_H__
#define __NORMAL_ESTIMATOR_H__

#include "data_structures/kdtree.h"
#include "pointcloud_lib/point_utils.h"

/**
* @brief Normal estimator class
*
* Given a pointcloud as a vector points, this object can be used to extract normals at every point.  
* Normal at a point is estimated based on negiborhood points around that point  
* Given a vector of points around a point in a small neighborhood (say within 0.1 m around the point), then surface normal 
* at that point can be estimated from `SVD` of the covariance matrix of the points vector  
* 
* For a set of n 3d points in the neighborhood of 0.1 m, then define marix `A` \f$\ni\f$  
* \f[A=\left[\begin{array}{A} x_0-\bar{x} & y_0-\bar{y} & z_0-\bar{z} \\
*		x_1-\bar{x} & y_1-\bar{y} & z_1-\bar{z} \\
*		... & ... & ... \\
*		x_n-\bar{x} & y_n-\bar{y} & z_n-\bar{z} \end{array}\right]
* \f]  
* where \f$\bar{x}=\frac{\Sigma{x_i}}{n}\f$, \f$\bar{y}=\frac{\Sigma{y_i}}{n}\f$, \f$\bar{z}=\frac{\Sigma{z_i}}{n}\f$  
* covariance matrix can be calculated as, \f$cov=A^T A\f$  
* then vector corresponding to the least singular value in the SVD of `cov` gives the corresponding surface normal
*/
template<unsigned int d, class T>
class NormalEstimator
{
	static_assert(d==3 && (std::is_same<T, float>::value || std::is_same<T, double>::value), "only supports 3 dimensional float and double points");
public:
	NormalEstimator(): _has_normals(false) {}

	void set_pointcloud(std::vector< Point<d, T> >& pointvec)
	{
		_pointvec = pointvec;
		_has_normals = false;
	}

	std::vector< Point<d, T> > get_normals(const double& search_radius)
	{
		if(!_has_normals) compute_normals(search_radius);
		return _normalvec;
	}

private:
	std::vector< Point<d, T> > _pointvec;
	std::vector< Point<d, T> > _normalvec;
	bool _has_normals;

	void compute_normals(const double& search_radius)
	{
		// O(nlogn) -> KDTree retreives neighbors in average O(logn) time
		if(_has_normals) return;

		_normalvec = std::vector< Point<d, T> >();
		auto _tmpvec = _pointvec;
		KDTree<d, T> _tree;
		_tree.build(_tmpvec);
		for(auto p: _pointvec)
		{
			auto pneighbors = _tree.neighborhood(p, search_radius);
			if(pneighbors.size()>=3)
			{
				// compute covariance matrix
				Eigen::Matrix<T, d, d> cov = compute_covariance_matrix(pneighbors);
				
				// SVD decomposition of cov into U*D*VT
				// normal vector is the column in V corresponding to least singular value of covariance matrix
				Eigen::JacobiSVD< Eigen::Matrix<T, d, d> > svd(cov, Eigen::DecompositionOptions::ComputeFullU | Eigen::DecompositionOptions::ComputeFullV);
				Eigen::Matrix<T, d, d> V = svd.matrixV();
				Point<d, T> normal;
				for(int i=0; i<d; ++i) normal[i] = V(i, d-1);

				// check for consistency in the direction
				// assume normals point towards origin
				normal = (normal.dot(-p)>0)?normal:-normal;

				_normalvec.push_back(normal);
			}
			else _normalvec.push_back(Point<d, T>());
		}
		_has_normals = true;
	}
};

#endif