# SFND 3D Object Tracking

## Flowchart of the Sensor Fusion framework for Camera + LiDAR in estimating Time to collision (TTC)

![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-Camera/blob/master/images/codeFlow.png)

This sensor fusion framework is an example of low level fusion.

This project has four major parts: 
1. First, matching 3D objects over time by using keypoint correspondences. 
2. Second, computing the TTC based on Lidar measurements. 
3. Computing TTC using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, conducting various tests with the framework. Goal here is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor.

## Detecting Keypoints and extacting Descriptors
* For keypoints detection following have been implemented in ```matching2D_Student.cpp```
  *HARRIS*, *SHITOMASI*, *FAST*, *BRISK*, *ORB*, *AKAZE*
* While for decriptors: *SIFT*, *BRISK*, *BRIEF*, *ORB*, *FREAK*
* For matching *BruteForce* and *FLANN* has been implemented. And to reduce false positives (FP) choice of two approches are implemented: *Nearest neighbor(NN)* and *Kth-Nearest neighbor(KNN)*

![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-Camera/blob/master/images/ptsMatch.jpg)

## Projection of 3D LiDAR point cloud on 2D Image Plane
![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-Camera/blob/master/images/eq_Lidar-cam.png)

Here, ```P ``` is a point in 3D plane which is projected on to the image plane by multiplying with *intrinsic* and *extrinsic* matrices, ``` P' ``` is the 2D projection of ```P```.

![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-Camera/blob/master/images/Cam-LidarProj.png)

## Object Detection with YOLO
![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-Camera/blob/master/images/yolo.png)

## Creating 3D objects
Now as we have bounding boxs from Yolo and we can find the 2D projected LiDAR points enclosed by it we can reverse and the equaations  discussed above and cluster 3D Lidar points. But for this we need to define a new ```struct``` so as to keep LiDAR point, keypoints and descriptor associated with bounding boxes together.

```
struct BoundingBox { // bounding box around a classified object (contains both 2D and 3D data)

    int boxID; // unique identifier for this bounding box
    int trackID; // unique identifier for the track to which this bounding box belongs

    cv::Rect roi; // 2D region-of-interest in image coordinates
    int classID; // ID based on class file provided to YOLO framework
    double confidence; // classification trust

    std::vector<LidarPoint> lidarPoints; // Lidar 3D points which project into 2D image roi
    std::vector<cv::KeyPoint> keypoints; // keypoints enclosed by 2D roi
    std::vector<cv::DMatch> kptMatches; // keypoint matches enclosed by 2D roi
};
```
![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-Camera/blob/master/images/LidarBBox.png)

## Estimating TTC using Camera
![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-Camera/blob/master/images/TTC_Cam_3.jpg)

In (a), a set of keypoints has been detected and the relative distances between keypoints 1-7 have been computed. In (b), 4 keypoints have been matched between successive images (with keypoint 3 being a mismatch) using descriptors. The ratio of all relative distances between each other can be used to compute a reliable TTC estimate.

## Estimating TTC using LiDAR
![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-Camera/blob/master/images/TTC_Lid_1.png)
![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-Camera/blob/master/images/TTC_Lid_2.png)
Here, constant velocity model is assumed for the target vehicle

# Final OUTPUT
![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-Camera/blob/master/images/Result.png)


## Dependencies
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
