#!/bin/bash

# navigate to build dir and use map.pgm to create plan use A*
cd ../build
./test_astar

# visualize generated path
cd ../scripts
python3 check_map_path.py