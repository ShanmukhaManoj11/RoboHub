RoboHub is a collection of C++ implementations of algorithms used in robotics

## Build and run content
### 1. Using docker (Recommended)
``` bash
# build content
$ ./docker_build.sh
# run docker interactvely
$ ./docker_run.sh
```
All the built content can be found in `/workspace/build/` directory when the docker terminal is up

### 2. Native build (dependencies might need to be installed separately)
``` bash
$ mkdir build && cd build
$ cmake ..
$ make
```
## Details

1. Currently this project doesn't use any of the pcl or ROS components. All imlementations are primarily based on STL and Eigen. Data generations and visualization for tests is made with python scripts (can be found in the `scripts` directory) that use `numpy` and `matplotlib` for 2d and `mayavi` for 3d visualizations

2. API is header only. Implementations can be found in `include` directory

3. All functionaly can be qualitatively and/or quantitatively tested using provided tests found in `tests` directory