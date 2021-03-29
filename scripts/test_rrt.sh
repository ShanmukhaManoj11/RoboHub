#!/bin/bash

# navigate to build dir and use map.pgm to create plan use RRT
cd ../build
./test_rrt

# visualize generated path
cd ../scripts
python3 check_map_path.py