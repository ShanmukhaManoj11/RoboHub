#include "planning_lib/rrt_planner.h"
#include <iostream>
#include <chrono>
#include <fstream>
#include <sstream>

struct Map_Data
{
	std::vector<unsigned char> data;
	int width;
	int height;
	int max_val;
};

Map_Data read_pgm(std::string mapfile)
{
	Map_Data map_data;

	std::ifstream f(mapfile.c_str(), std::ios::binary);
	std::stringstream ss;
	std::string line = "";

	getline(f, line);
	// std::cout<<line<<std::endl;

	getline(f, line);
	// std::cout<<line<<std::endl;

	ss << f.rdbuf();
	ss >> map_data.width >> map_data.height >> map_data.max_val;
	
	map_data.data.resize(map_data.width*map_data.height);
	for(int i=0; i<map_data.height; ++i)
	{
		for(int j=0; j<map_data.width; ++j)
		{
			ss >> map_data.data[i*map_data.width+j];
		}
	}

	return map_data;
}

std::vector<int> raw_pgm_to_occupancy_data(Map_Data& map_data, float occupied_thresh, float free_thresh)
{
	std::vector<int> occ_data(map_data.width*map_data.height, -1);
	for(int i=0; i<map_data.height; ++i)
	{
		for(int j=0; j<map_data.width; ++j)
		{
			float prob = float(255-int(map_data.data[i*map_data.width+j]))/255.0;
			if(prob > occupied_thresh) occ_data[i*map_data.width+j] = 100;
			else if(prob < free_thresh) occ_data[i*map_data.width+j] = 0;
		}
	}
	return occ_data;
}

void save_path_as_bin(std::vector<Point2f>& path, std::string outfile)
{
	std::ofstream f(outfile.c_str(), std::ios::binary);

	float* data = new float[2];
	for(auto point: path)
	{
		for(int i=0; i<2; ++i) data[i] = point[i];
		f.write((char *)data, 2*sizeof(float));
	}
	delete[] data;
	f.close();
}

void test_rrt()
{
	std::cout<<"rrt on known map"<<std::endl;
	Map_Data map_data = read_pgm("../data/map.pgm");
	float occupied_thresh = 0.65, free_thresh = 0.196;
	auto occ_data = raw_pgm_to_occupancy_data(map_data, occupied_thresh, free_thresh);

	RRTPlanner2d planner;

	float resolution = 0.02;
	Point2f origin({-33.0, -39.4});
	planner.set_map(occ_data, resolution, map_data.width, map_data.height, origin);

	Point2f start({11.0, -9.4});
	Point2f goal({7.0, -5.4});
	auto start_t = std::chrono::high_resolution_clock::now();
	auto path = planner.compute_plan(start, goal);
	auto end_t = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> duration = end_t-start_t;
	std::cout<<"\tcompute time: "<<duration.count()<<"s"<<std::endl;

	save_path_as_bin(path, "../data/map_path.bin");
}

int main(int argc, char** argv)
{
	test_rrt();
}