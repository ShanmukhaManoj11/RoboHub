#include "data_structures/point_types.h"

#include <iostream>

int main(int argc, char** argv)
{
	Point<2, int> p1;
	p1[1] = 8;
	std::cout<<p1[0]<<" "<<p1[1]<<std::endl;

	Point3i p2({-1, 9, 19});
	std::cout<<p2[0]<<" "<<p2[1]<<" "<<p2[2]<<std::endl;

	Point3f p3({21.0, -9.8, 7.66});
	std::cout<<p3[0]<<" "<<p3[1]<<" "<<p3[2]<<std::endl;

	Point3f p4(p3);
	p4[1] = 1.288;
	std::cout<<p4[0]<<" "<<p4[1]<<" "<<p4[2]<<std::endl;
}