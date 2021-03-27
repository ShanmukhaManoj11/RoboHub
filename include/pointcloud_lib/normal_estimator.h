#ifndef __NORMAL_ESTIMATOR_H__
#define __NORMAL_ESTIMATOR_H__

#include "data_structures/kdtree.h"
#include "pointcloud_lib/point_utils.h"

template<unsigned int d, class T>
class NormalEstimator
{
	static_assert(d==3 && (std::is_same<T, float>::value || std::is_same<T, double>::value), "only supports float and double points");
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