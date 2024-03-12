# SuMa++: Efficient LiDAR-based Semantic SLAM

This repository contains the implementation of SuMa++, which generates semantic maps only using three-dimensional laser range scans.

Developed by [Xieyuanli Chen](https://www.ipb.uni-bonn.de/people/xieyuanli-chen/) and [Jens Behley](https://www.ipb.uni-bonn.de/people/jens-behley/).

SuMa++ is built upon SuMa and RangeNet++. For more details, we refer to the original project websites [SuMa](https://github.com/jbehley/SuMa) and [RangeNet++](https://github.com/PRBonn/lidar-bonnetal).

_An example of using SuMa++:_
![ptcl](./.readme/demo.gif)


### Table of Contents
0. [Introduction](#SuMa++:-Efficient-LiDAR-based-Semantic-SLAM)
1. [Publication](#Publication)
2. [Docker](#Docker)
3. [Dependencies](#Dependencies)
4. [Build](#Build)
5. [How to run](#How-to-run)
6. [More Related Work](#More-Related-Work)
7. [Frequently Asked Questions](#Frequently-Asked-Questions)
8. [License](#License)

## Publication

If you use our implementation in your academic work, please cite the corresponding [paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/chen2019iros.pdf):  
    
	@inproceedings{chen2019iros, 
			author = {X. Chen and A. Milioto and E. Palazzolo and P. Giguère and J. Behley and C. Stachniss},
			title  = {{SuMa++: Efficient LiDAR-based Semantic SLAM}},
			booktitle = {Proceedings of the IEEE/RSJ Int. Conf. on Intelligent Robots and Systems (IROS)},
  			pages = {4530--4537},
			year = {2019},
			url = {https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/chen2019iros.pdf}
	}
 
## Docker

Thanks to the efforts of [Hyunggi Chang](https://github.com/changh95), we have now a dockerized version of `semantic_suma`, which takes care of the proper setup.

You can build the Docker image with the provided Dockerfile, i.e.,
```bash
docker build -t semantic_suma:latest
```

and run the container by 
```bash
docker run -it -e "DISPLAY=$DISPLAY" -e "QT_X11_NO_MITSHM=1" -e "XAUTHORITY=$XAUTH" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v /kitti:/data --runtime=nvidia --net=host --ipc=host --privileged semantic_suma:latest
```

Note that it makes a volume located at `kitti` that should point to the KITTI files or another directory containing scans in the KITTI format. Follow the [How to run](#How-to-run) instructions below to execute the binary.

The GUI is visualized via X11, so prior to starting a docker container, `xhost +local:docker` command should be used to provide access to the xhost from docker.

## Dependencies

* catkin
* Qt5 >= 5.2.1
* OpenGL >= 4.0
* libEigen >= 3.2
* gtsam >= 4.0  (tested with [4.0.0-alpha2](https://github.com/borglab/gtsam/tree/4.0.0-alpha2))

In Ubuntu 16.04: Installing all dependencies should be accomplished by
```bash
sudo apt-get install build-essential cmake libgtest-dev libeigen3-dev libboost-all-dev qtbase5-dev libglew-dev libqt5libqgtk2 catkin
```

Additionally, make sure you have [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/) and the [fetch](https://github.com/Photogrammetry-Robotics-Bonn/catkin_tools_fetch) verb installed:
```bash
sudo apt install python-pip
sudo pip install catkin_tools catkin_tools_fetch empy
```

## Build
#### rangenet_lib
To use SuMa++, you need to first build the rangenet_lib with the TensorRT and C++ interface. 
For more details about building and using rangenet_lib you could find in [rangenet_lib](https://github.com/PRBonn/rangenet_lib).

#### SuMa++
Clone the repository in the `src` directory of the same catkin workspace where you built the rangenet_lib:
```bash
git clone https://github.com/PRBonn/semantic_suma.git
```
Download the additional dependencies (or clone [glow](https://github.com/jbehley/glow.git) into your catkin workspace `src` yourself):
```bash
catkin deps fetch
```

For the first setup of your workspace containing this project, you need:
  ```bash
catkin build --save-config -i --cmake-args -DCMAKE_BUILD_TYPE=Release -DOPENGL_VERSION=430 -DENABLE_NVIDIA_EXT=YES
  ```
  
  Where you have to set `OPENGL_VERSION` to the supported OpenGL core profile version of your system, which you can query as follows:

```bash
$ glxinfo | grep "version"
server glx version string: 1.4
client glx version string: 1.4
GLX version: 1.4
OpenGL core profile version string: 4.3.0 NVIDIA 367.44
OpenGL core profile shading language version string: 4.30 NVIDIA [...]
OpenGL version string: 4.5.0 NVIDIA 367.44
OpenGL shading language version string: 4.50 NVIDIA
```

 Here the line `OpenGL core profile version string: 4.3.0 NVIDIA 367.44` is important and therefore you should use `-DOPENGL_VERSION = 430`. If you are unsure you can also leave it on the default version `330`, which should be supported by all OpenGL-capable devices.

 If you have a NVIDIA device, like a Geforce or Quadro graphics card, you should also activate the NVIDIA extensions using `-DENABLE_NVIDIA_EXT=YES` for info about the current GPU memory usage of the program.

 After this setup steps, you can build with `catkin build`, since the configuration has been saved to your current Catkin profile (therefore, `--save-config` was needed).
 
 Now the project root directory (e.g. `~/catkin_ws/src/semantic_suma`) should contain a `bin` directory containing the visualizer.

## How to run
**Important Notice**
- Before running SuMa++, you need to first build the [rangenet_lib](https://github.com/PRBonn/rangenet_lib) and download the pretrained [model](https://www.ipb.uni-bonn.de/html/projects/semantic_suma/darknet53.tar.gz).
- You need to specify the model path in the configuration file in the `config/` folder.
- For the first time using, rangenet_lib will take several minutes to build a `.trt` model for SuMa++.
- SuMa++ now can only work with KITTI dataset, since the semantic segmentation may not generalize well in other environments.
- To use SuMa++ with your own dataset, you may finetune or retrain the semantic segmentation network.

All binaries are copied to the `bin` directory of the source folder of the project. Thus,
1. run `visualizer` in the `bin` directory by `./visualizer`,
2. open a Velodyne directory from the KITTI Visual Odometry Benchmark and select a ".bin" file,
3. start the processing of the scans via the "play button" in the GUI.

## More Related Work

### [OverlapNet - Loop Closing for 3D LiDAR-based SLAM](https://github.com/PRBonn/OverlapNet)
This repo contains the code for our RSS2020 paper: OverlapNet - Loop Closing for 3D LiDAR-based SLAM.

OverlapNet is a modified Siamese Network that predicts the overlap and relative yaw angle of a pair of range images generated by 3D LiDAR scans, which can be used for place recognition and loop closing.

### [Overlap-based LiDAR Global Localization](https://github.com/PRBonn/overlap_localization)
This repo contains the code for our IROS2020 paper: Learning an Overlap-based Observation Model for 3D LiDAR Localization.

It uses the OverlapNet to train an observation model for Monte Carlo Localization and achieves global localization with 3D LiDAR scans.


## Frequently Asked Questions
- [visualizer segmentation fault](https://github.com/PRBonn/semantic_suma/issues/5)
- [Different semantic results of suma++](https://github.com/PRBonn/semantic_suma/issues/6)
- [How do I save the global map?](https://github.com/PRBonn/semantic_suma/issues/54)

## License


Copyright 2019, Xieyuanli Chen, Jens Behley, Cyrill Stachniss, Photogrammetry and Robotics Lab, University of Bonn.

This project is free software made available under the MIT License. For details see the LICENSE file.


