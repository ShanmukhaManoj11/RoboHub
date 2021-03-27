#include "pointcloud_lib/normal_estimator.h"
#include <iostream>
#include <fstream>
#include <chrono>

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

void test_normal_estimation_plane()
{
	std::cout<<"normal estimation - plane"<<std::endl;
	std::string binfile = "../data/plane.bin";
	std::vector<Point3f> pointvec = read_bin<3, float>(binfile);
	std::cout<<"\t"<<pointvec.size()<<" points"<<std::endl;

	NormalEstimator<3, float> ne;
	ne.set_pointcloud(pointvec);
	double radius = 0.2;
	auto start = std::chrono::high_resolution_clock::now();
	std::vector<Point3f> normalvec = ne.get_normals(radius);
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> duration = end-start;
	std::cout<<"\tcompute time: "<<duration.count()<<"s"<<std::endl;

	std::string outfile = "../data/plane_normals.bin";
	save_bin(normalvec, outfile);
	std::cout<<"\tnormals saved to "<<outfile<<std::endl;
}

void test_normal_estimation_sphere()
{
	std::cout<<"normal estimation - sphere"<<std::endl;
	std::string binfile = "../data/sphere.bin";
	std::vector<Point3f> pointvec = read_bin<3, float>(binfile);
	std::cout<<"\t"<<pointvec.size()<<" points"<<std::endl;

	NormalEstimator<3, float> ne;
	ne.set_pointcloud(pointvec);
	double radius = 0.2;
	auto start = std::chrono::high_resolution_clock::now();
	std::vector<Point3f> normalvec = ne.get_normals(radius);
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> duration = end-start;
	std::cout<<"\tcompute time: "<<duration.count()<<"s"<<std::endl;

	std::string outfile = "../data/sphere_normals.bin";
	save_bin(normalvec, outfile);
	std::cout<<"\tnormals saved to "<<outfile<<std::endl;
}

void test_normal_estimation_kitti()
{
	std::cout<<"normal estimation - kitti sample"<<std::endl;
	std::string binfile = "../data/0000000000.bin";
	std::vector<Point3f> pointvec = read_kitti_bin(binfile);
	std::cout<<"\t"<<pointvec.size()<<" points"<<std::endl;

	NormalEstimator<3, float> ne;
	ne.set_pointcloud(pointvec);
	double radius = 0.2;
	auto start = std::chrono::high_resolution_clock::now();
	std::vector<Point3f> normalvec = ne.get_normals(radius);
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> duration = end-start;
	std::cout<<"\tcompute time: "<<duration.count()<<"s"<<std::endl;

	std::string outfile = "../data/0000000000_normals.bin";
	save_bin(normalvec, outfile);
	std::cout<<"\tnormals saved to "<<outfile<<std::endl;
}

int main(int argc, char** argv)
{
	test_normal_estimation_plane();
	test_normal_estimation_sphere();
	// test_normal_estimation_kitti();
}