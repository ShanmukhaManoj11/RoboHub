#include "pointcloud_lib/plane_extractor.h"
#include <iostream>
#include <fstream>
#include <chrono>

std::vector<Point3f> read_kitti_bin(const std::string& binfile)
{
	std::vector<Point3f> pointvec;
	std::ifstream f(binfile.c_str(), std::ios::binary);

	float* data = new float[4];
	while(f.read((char *)data, 4*sizeof(float)))
	{
		Point3f point;
		for(int i=0; i<3; ++i) point[i] = data[i];
		pointvec.push_back(point);
	}

	delete[] data;
	f.close();
	return pointvec;
}

template<unsigned int d, class T>
void save_bin(const std::vector< Point<d, T> >& pointvec, std::string outfile)
{
	std::ofstream f(outfile.c_str(), std::ios::binary);

	T *data = new T[d];
	for(auto point: pointvec)
	{
		for(int i=0; i<d; ++i) data[i] = point[i];
		f.write((char *)data, d*sizeof(T));
	}

	delete[] data;
	f.close();
}

void test_plane_extraction()
{
	std::cout<<"plane extraction - kitti sample"<<std::endl;
	std::string binfile = "../data/0000000000.bin";
	std::vector<Point3f> pointvec = read_kitti_bin(binfile);
	std::cout<<"\t"<<pointvec.size()<<" points"<<std::endl;

	PlaneExtractor<float> pe;

	auto start = std::chrono::high_resolution_clock::now();
	auto points_pair = pe.extract_plane(pointvec, 100, 0.3);
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> duration = end-start;
	std::cout<<"\tcompute time: "<<duration.count()<<"s"<<std::endl;

	std::string outfile = "../data/0000000000_plane_points.bin";
	save_bin(points_pair.first, outfile);
	std::cout<<"\tplane points saved to "<<outfile<<std::endl;

	outfile = "../data/0000000000_other_points.bin";
	save_bin(points_pair.second, outfile);
	std::cout<<"\tother points saved to "<<outfile<<std::endl;
}

int main(int argc, char** argv)
{
	test_plane_extraction();
}