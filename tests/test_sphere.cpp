#include "pointcloud_lib/normal_estimator.h"
#include <iostream>
#include <fstream>

template<unsigned int d, class T>
std::vector< Point<d, T> > read_bin(const std::string& binfile)
{
	std::vector< Point<d, T> > pointvec;
	std::ifstream f(binfile.c_str(), std::ios::binary);

	T *data = new T[d];
	while(f.read((char *)data, d*sizeof(T)))
	{
		Point<d, T> point;
		for(int i=0; i<d; ++i) point[i] = data[i];
		pointvec.push_back(point);
	}

	delete[] data;
	f.close();
	return pointvec;
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

int main(int argc, char** argv)
{
	auto comp = [](Point3f& a, Point3f& b){
		if(a[0]==b[0]) 
		{
			if(a[1]==b[1]) return a[2]<b[2];
			return a[1]<b[1];
		}
		return a[0]<b[0];
	};

	std::string binfile = "/home/mano/work/robotics_ws/data/sphere.bin";
	std::vector<Point3f> pointvec = read_bin<3, float>(binfile);
	
	for(int i=0; i<pointvec.size(); i += 100)
	{
		auto qpoint = pointvec[i];
		double radius = 0.2;

		auto neighbors_bf = neighborhood_bruteforce(pointvec, qpoint, radius);
		sort(neighbors_bf.begin(), neighbors_bf.end(), comp);
		// for(auto point: neighbors_bf) std::cout<<"("<<point[0]<<","<<point[1]<<","<<point[2]<<"), ";
		// std::cout<<std::endl;

		KDTree<3, float> tree;
		tree.build(pointvec);
		auto neighbors = tree.neighborhood(qpoint, radius);
		sort(neighbors.begin(), neighbors.end(), comp);
		// for(auto point: neighbors) std::cout<<"("<<point[0]<<","<<point[1]<<","<<point[2]<<"), ";
		// std::cout<<std::endl;

		assert(neighbors_bf.size() == neighbors.size());
		for(int i=0; i<neighbors.size(); ++i) assert(neighbors[i].is_equal_to(neighbors_bf[i]));
		std::cout<<"query point "<<i<<" - passed"<<std::endl;
	}
}