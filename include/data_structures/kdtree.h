#ifndef __KDTREE_H__
#define __KDTREE_H__

#include "data_structures/point_types.h"
#include <memory>
#include <algorithm>

/**
* @brief KDNode template class defining a node in a KDTree
*/
template<unsigned int d, class T>
class KDNode
{
public:
	typedef std::shared_ptr<KDNode> KDNodePtr;

	Point<d, T> point;
	KDNodePtr left;
	KDNodePtr right;

	KDNode(): point(), left(nullptr), right(nullptr) {}
	KDNode(Point<d, T>& p): point(p), left(nullptr), right(nullptr) {}
};

/**
* @brief KDTree template class
* 
* KDTree can be used to store k-dimensional data points and efficiently perform search operations  
* For example 3 dimensional real value points can be stored in a `KDTree<3, float>` type tree
*/
template<unsigned int d, class T>
class KDTree
{
public:
	typedef std::shared_ptr< KDNode<d, T> > KDNodePtr;
	
	KDTree(): root(nullptr) {}

	/**
	* @brief build tree from a vector of points
	*
	* @param pointvec vector of points
	*/
	void build(std::vector< Point<d, T> >& pointvec)
	{
		root = build_tree_recursive(pointvec, 0, pointvec.size(), 0);
	}

	/**
	* @brief retreive the closest point in the tree to a query point
	*
	* @param point query point
	*
	* @return returns a point in the tree that is closest to the query point
	*
	* @note if multiple closest points exist only one of them is returned. If all the closest points are needed, retrieve them using neighborhood with input search radius = distance to this closest point
	*/
	Point<d, T> search(const Point<d, T>& point)
	{
		KDNodePtr rnode = nullptr;
		double min_dist = 0.0;
		search_closest(root, point, 0, rnode, min_dist);
		return rnode->point;
	}

	/**
	* @brief get nodes in the tree that are with in radius units from the query point
	*
	* @param point query point
	* @param radius search radius
	*
	* @return returns vector of node pointers that are within radius units from the input query point
	*/
	std::vector< Point<d, T> > neighborhood(const Point<d, T>& point, const double& radius)
	{
		std::vector< Point<d, T> > neighbors;
		neighborhood_recursive(root, point, radius, 0, neighbors);
		return neighbors;
	}

private:
	KDNodePtr root;

	/**
	* @brief recursive helper function to build tree from a point vector
	*
	* @param pointvec vector of points
	* @param l start index of the vector
	* @param r end index the vector
	* @param id current search dimension
	* 
	* @note builds tree using the vector between indices [l, r)
	*/
	KDNodePtr build_tree_recursive(std::vector< Point<d, T> >& pointvec, int l, int r, int id)
	{
		if(r<=l) return nullptr;
		int m = l+(r-l)/2;
		// O(n) partition such that elements to the left are < than element at pivot position and elements to the right are >= to element at pivot position
		std::nth_element(pointvec.begin()+l, pointvec.begin()+m, pointvec.begin()+r, [&id](Point<d, T>& a, Point<d, T>& b){
			return a[id]<b[id];
		});
		KDNodePtr cur_root = std::make_shared< KDNode<d, T> >(pointvec[m]);
		id = (id+1)%d;
		// recursively build tree for left and right branches
		cur_root->left = build_tree_recursive(pointvec, l, m, id);
		cur_root->right = build_tree_recursive(pointvec, m+1, r, id);
		return cur_root;
	}

	/**
	* @brief recursive helper function to search for closest point in the tree to the query point 
	*
	* @param cur_root current root to recurse from
	* @param point query point
	* @param id current search dimension
	* @param rnode probable result node, it gets updated based on current distance
	* @param min_dist probable minimum distance to the query point from any node in the tree
	*/
	void search_closest(KDNodePtr& cur_root, const Point<d, T>& point, int id, KDNodePtr& rnode, double& min_dist)
	{
		if(cur_root==nullptr) return;
		double dist = point.distance_to(cur_root->point);
		// update minimum distance and result node if current distance < current minimum distance
		if(rnode==nullptr || dist < min_dist)
		{
			min_dist = dist;
			rnode = cur_root;
		}
		if(dist==0) return; // exact node is found
		// check if a closer point can exist in left branch
		if(point[id]-min_dist <= cur_root->point[id]) search_closest(cur_root->left, point, (id+1)%d, rnode, min_dist);
		// check if a closer point can exist in right branch
		if(point[id]+min_dist >= cur_root->point[id]) search_closest(cur_root->right, point, (id+1)%d, rnode, min_dist);
	}

	/**
	* @brief recursive helper function to retreive points in the tree that are within a radius of the query point
	*
	* @param cur_root current root to recurse from
	* @param point query point
	* @param radius search radius around the query point 
	* @param id current search dimension
	* @param neighbors vector of neighbors in the search area of the query point
	*/
	void neighborhood_recursive(KDNodePtr& cur_root, const Point<d, T>& point, const double& radius, int id, std::vector< Point<d, T> >& neighbors)
	{
		if(cur_root==nullptr) return;
		// if distance between cur_root and query point <= radius, add cur_root to neighbors
		double dist = point.distance_to(cur_root->point);
		if(dist<=radius) neighbors.push_back(cur_root->point);
		// check if neighbor can exist in left branch
		if(point[id]-radius <= cur_root->point[id]) neighborhood_recursive(cur_root->left, point, radius, (id+1)%d, neighbors);
		// check if neighbor can exist in right branch
		if(point[id]+radius >= cur_root->point[id]) neighborhood_recursive(cur_root->right, point, radius, (id+1)%d, neighbors);
	}
};

#endif