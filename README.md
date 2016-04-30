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
