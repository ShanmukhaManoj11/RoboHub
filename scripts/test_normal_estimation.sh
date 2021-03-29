#!/bin/bash

# generate points as .bin files
python3 generate_points.py

# use .bin file to compute normals and save as .bin file
cd ../build
./test_normal_estimation

# visualize generated normals
cd ../scripts
python3 check_normals.py