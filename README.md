# SFND 3D Object Tracking

## Flowchart of the Sensor Fusion framework for Camera + LiDAR in estimating Time to collision (TTC)

![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-Camera/blob/master/images/codeFlow.png)

This project has four major parts: 
1. First, matching 3D objects over time by using keypoint correspondences. 
2. Second, computing the TTC based on Lidar measurements. 
3. Computing TTC using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, conducting various tests with the framework. Goal here is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor.

## Detecting Keypoints and extacting Descriptors
For keypoints detection following have been implemented in ```matching2D_Student.cpp```
  "HARRIS", "SHITOMASI",  "FAST", "BRISK","ORB", "AKAZE"
While for decriptors: "SIFT",BRISK", "BRIEF", "ORB", "FREAK"
For matching BruteForce and FLANN has been implemented. And to reduce false positives (FP) choice of two approches are implemented: Nearest neighbor and Kth-Nearest neighbor

![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-Camera/blob/master/images/ptsMatch.jpg)

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
