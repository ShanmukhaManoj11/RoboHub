#!/bin/bash

# navigate to build dir and use sample KITTI pointcloud save as .bin file to extract plane
cd ../build
./test_plane_extraction

# visualize segmented pointcloud
cd ../scripts
python3 check_plane_extraction.py