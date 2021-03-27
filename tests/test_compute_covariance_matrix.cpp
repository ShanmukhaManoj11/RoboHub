#include "pointcloud_lib/point_utils.h"
#include <iostream>

/*
example test case:
input 3d points as 3x5 matrix
[[-0.80454205  0.72961361  0.94452929  0.1541065  -0.95911572]
 [-0.66212128 -0.97819251  0.92807165 -0.63307186 -0.09955121]
 [ 0.21951934 -0.27793888 -0.59808864 -0.96163355  0.07309721]]
resulting covariance matrix
[[ 0.60291537  0.14243572 -0.22853121]
 [ 0.14243572  0.4499491  -0.05470196]
 [-0.22853121 -0.05470196  0.18715971]]
*/
void test_1()
{
	std::vector<Point3d> pointvec;
	pointvec.push_back(Point3d({-0.80454205, -0.66212128, 0.21951934}));
	pointvec.push_back(Point3d({0.72961361, -0.97819251, -0.27793888}));
	pointvec.push_back(Point3d({0.94452929, 0.92807165, -0.59808864}));
	pointvec.push_back(Point3d({0.1541065, -0.63307186, -0.96163355}));
	pointvec.push_back(Point3d({-0.95911572, -0.09955121, 0.07309721}));

	Eigen::Matrix3d expected_cov;
	expected_cov << 0.60291537, 0.14243572, -0.22853121, 0.14243572, 0.4499491, -0.05470196, -0.22853121, -0.05470196, 0.18715971;

	Eigen::Matrix3d cov = compute_covariance_matrix(pointvec);
	
	for(int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j) assert(abs(expected_cov(i,j)-cov(i,j))<0.001);
	}
	std::cout<<"compute covariance matrix: test 1 passed"<<std::endl;
}

int main(int argc, char** argv)
{
	test_1();
}