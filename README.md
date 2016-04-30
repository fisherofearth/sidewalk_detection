# Sidewalk Detection

## Environment
Ubuntu 14.04
Intel_i5 8GB HHD

## Data
Download the [rosbag file](https://s3-us-west-1.amazonaws.com/marble-coding-challenges/coding_challenges_1/realsense_coding_challenge_1.bag)

## Try the example

  * Extract this package to a catkin workspace.
  * Add the package to catkin path
  * Run:
  ````
    roslaunch sidewalk_detection sidewalk_detector_ImageView.launch 
  ````
    or
  ````
    roslaunch sidewalk_detection sidewalk_detector_RVIZ.launch
  ````
  * In another terminal play the rosbag.

## Result
![result](https://github.com/fisherofearth/sidewalk_detection/blob/master/result/1.png)
![result](https://github.com/fisherofearth/sidewalk_detection/blob/master/result/2.png)
![result](https://github.com/fisherofearth/sidewalk_detection/blob/master/result/3.png)
![result](https://github.com/fisherofearth/sidewalk_detection/blob/master/result/4.png)
