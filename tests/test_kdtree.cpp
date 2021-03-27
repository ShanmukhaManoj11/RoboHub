#include "data_structures/kdtree.h"

#include <iostream>
#include <random>
#include <chrono>
#include <cassert>

template<unsigned int d, class T>
std::vector< Point<d, T> > search_closest_bruteforce(const std::vector< Point<d, T> >& pointvec, const Point<d, T>& qpoint)
{
	double min_dist = pointvec[0].distance_to(qpoint);
	for(int i=1; i<pointvec.size(); ++i) min_dist = std::min(min_dist, pointvec[i].distance_to(qpoint));

	std::vector< Point<d, T> > respoints;
	for(auto point: pointvec)
	{
		if(point.distance_to(qpoint)==min_dist) respoints.push_back(point);
	}
	return respoints;
}

template<unsigned int d, class T>
std::vector< Point<d, T> > neighborhood_bruteforce(const std::vector< Point<d, T> >& pointvec, const Point<d, T>& qpoint, const double& radius)
{
	std::vector< Point<d, T> > neighbors;
	for(auto point: pointvec)
	{
		if(point.distance_to(qpoint)<=radius) neighbors.push_back(point);
	}
	return neighbors;
}

void test_2i_search()
{
	std::random_device rd;
	std::mt19937 gen(rd());

	// 2d int points
	std::uniform_int_distribution<> distrib(-100, 100);
	int npoints = 1000;
	std::vector<Point2i> point2i_vec;
	for(int i=0; i<npoints; ++i) point2i_vec.push_back(Point2i({distrib(gen), distrib(gen)}));
	Point2i qpoint({distrib(gen), distrib(gen)});

	auto start = std::chrono::high_resolution_clock::now();
	std::vector<Point2i> respoints_bf = search_closest_bruteforce(point2i_vec, qpoint);
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> duration = end-start;
	std::cout<<"bruteforce::search closest process time: "<<duration.count()<<"s"<<std::endl;
	for(auto point: respoints_bf) std::cout<<"("<<point[0]<<","<<point[1]<<"), ";
	std::cout<<std::endl;

	KDTree<2, int> tree;
	tree.build(point2i_vec);
	start = std::chrono::high_resolution_clock::now();
	Point2i cpoint = tree.search(qpoint);
	end = std::chrono::high_resolution_clock::now();
	duration = end-start;
	std::cout<<"kdtree::search closest process time: "<<duration.count()<<"s"<<std::endl;
	std::cout<<"("<<cpoint[0]<<","<<cpoint[1]<<")"<<std::endl;
}

void test_3f_search()
{
	std::random_device rd;
	std::mt19937 gen(rd());

	// 3d float points
	std::uniform_real_distribution<float> distrib(-100.0, 100.0);
	int npoints = 10000;
	std::vector<Point3f> point3f_vec;
	for(int i=0; i<npoints; ++i) point3f_vec.push_back(Point3f({distrib(gen), distrib(gen), distrib(gen)/10.0f}));
	Point3f qpoint({distrib(gen), distrib(gen), distrib(gen)/10.0f});

	auto start = std::chrono::high_resolution_clock::now();
	std::vector<Point3f> respoints_bf = search_closest_bruteforce(point3f_vec, qpoint);
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> duration = end-start;
	std::cout<<"bruteforce::search closest process time: "<<duration.count()<<"s"<<std::endl;
	for(auto point: respoints_bf) std::cout<<"("<<point[0]<<","<<point[1]<<","<<point[2]<<"), ";
	std::cout<<std::endl;

	KDTree<3, float> tree;
	tree.build(point3f_vec);
	start = std::chrono::high_resolution_clock::now();
	Point3f cpoint = tree.search(qpoint);
	end = std::chrono::high_resolution_clock::now();
	duration = end-start;
	std::cout<<"kdtree::search closest process time: "<<duration.count()<<"s"<<std::endl;
	std::cout<<"("<<cpoint[0]<<","<<cpoint[1]<<","<<cpoint[2]<<")"<<std::endl;
}

void test_2i_neigborhood()
{
	std::random_device rd;
	std::mt19937 gen(rd());

	// 2d int points
	std::uniform_int_distribution<> distrib(-100, 100);
	int npoints = 1000;
	std::vector<Point2i> point2i_vec;
	for(int i=0; i<npoints; ++i) point2i_vec.push_back(Point2i({distrib(gen), distrib(gen)}));
	Point2i qpoint({distrib(gen), distrib(gen)});

	double radius=10.0;

	auto start = std::chrono::high_resolution_clock::now();
	std::vector<Point2i> neighbors_bf = neighborhood_bruteforce(point2i_vec, qpoint, radius);
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> duration = end-start;
	// std::cout<<"bruteforce::neighborhood process time: "<<duration.count()<<"s"<<std::endl;
	sort(neighbors_bf.begin(), neighbors_bf.end(), [](Point2i& a, Point2i& b){
		if(a[0]==b[0]) return a[1]<b[1];
		return a[0]<b[0];
	});
	// for(auto point: neighbors_bf) std::cout<<"("<<point[0]<<","<<point[1]<<"), ";
	// std::cout<<std::endl;

	KDTree<2, int> tree;
	tree.build(point2i_vec);
	start = std::chrono::high_resolution_clock::now();
	auto neighbors = tree.neighborhood(qpoint, radius);
	end = std::chrono::high_resolution_clock::now();
	duration = end-start;
	// std::cout<<"kdtree::neighborhood process time: "<<duration.count()<<"s"<<std::endl;
	sort(neighbors.begin(), neighbors.end(), [](Point2i& a, Point2i& b){
		if(a[0]==b[0]) return a[1]<b[1];
		return a[0]<b[0];
	});
	// for(auto node: neighbors) std::cout<<"("<<node->point[0]<<","<<node->point[1]<<"), ";
	// std::cout<<std::endl;

	assert(neighbors_bf.size()==neighbors.size());
	for(int i=0; i<neighbors_bf.size(); ++i)
	{
		for(int j=0; j<2; ++j) assert(neighbors_bf[i][j]==neighbors[i][j]);
	}
	std::cout<<"neighborhood search test for 2 dimensional int points passed"<<std::endl;
}

void test_3f_neigborhood()
{
	std::random_device rd;
	std::mt19937 gen(rd());

	// 3d float points
	std::uniform_real_distribution<float> distrib(-10.0, 10.0);
	int npoints = 10000;
	std::vector<Point3f> point3f_vec;
	for(int i=0; i<npoints; ++i) point3f_vec.push_back(Point3f({distrib(gen), distrib(gen), distrib(gen)/10.0f}));
	// Point3f qpoint({distrib(gen), distrib(gen), distrib(gen)/10.0f});
	for(int i=0; i<1; i += 1)
	{
		// auto qpoint = point3f_vec[i];
		Point3f qpoint({distrib(gen), distrib(gen), distrib(gen)/10.0f});

		double radius=1.0;

		auto start = std::chrono::high_resolution_clock::now();
		std::vector<Point3f> neighbors_bf = neighborhood_bruteforce(point3f_vec, qpoint, radius);
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> duration = end-start;
		// std::cout<<"bruteforce::neighborhood process time: "<<duration.count()<<"s"<<std::endl;
		sort(neighbors_bf.begin(), neighbors_bf.end(), [](Point3f& a, Point3f& b){
			if(a[0]==b[0]) 
			{
				if(a[1]==b[1]) return a[2]<b[2];
				return a[1]<b[1];
			}
			return a[0]<b[0];
		});
		// for(auto point: neighbors_bf) std::cout<<"("<<point[0]<<","<<point[1]<<","<<point[2]<<"), ";
		// std::cout<<std::endl;

		KDTree<3, float> tree;
		tree.build(point3f_vec);
		start = std::chrono::high_resolution_clock::now();
		auto neighbors = tree.neighborhood(qpoint, radius);
		end = std::chrono::high_resolution_clock::now();
		duration = end-start;
		// std::cout<<"kdtree::neighborhood process time: "<<duration.count()<<"s"<<std::endl;
		sort(neighbors.begin(), neighbors.end(), [](Point3f& a, Point3f& b){
			if(a[0]==b[0]) 
			{
				if(a[1]==b[1]) return a[2]<b[2];
				return a[1]<b[1];
			}
			return a[0]<b[0];
		});
		// for(auto node: neighbors) std::cout<<"("<<node->point[0]<<","<<node->point[1]<<","<<node->point[2]<<"), ";
		// std::cout<<std::endl;

		assert(neighbors_bf.size()==neighbors.size());
		for(int i=0; i<neighbors_bf.size(); ++i)
		{
			for(int j=0; j<2; ++j) assert(neighbors_bf[i][j]==neighbors[i][j]);
		}
	}
	std::cout<<"neighborhood search test for 3 dimensional float points passed"<<std::endl;
}

int main(int argc, char** argv)
{
	test_2i_search();
	test_3f_search();

	test_2i_neigborhood();
	test_3f_neigborhood();
}