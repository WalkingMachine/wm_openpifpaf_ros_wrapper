# wm_openpifpaf_ros_wrapper
## Description
ROS wrapper for openpifpaf pose detection. We currently use the web server used in openpifpafwebdemo to make http calls. We send the image coming from the throttle down topic /pose_detection/image_in (1 fps).

## Dependencies
* https://github.com/vita-epfl/openpifpaf
* https://github.com/vita-epfl/openpifpafwebdemo
## Installation
* clone openpifpafwebdemo outside your ROS repo
* Follow openpifpaf and openpifpafwebdemo installation instructions
## Utilization
* ```roslaunch openni2_launch openni2.launch```
* ```rosrun topic_tools throttle messages /camera/rgb/image_raw 1 /pose_detection/image_in```
* ```python3.7 -m openpifpafwebdemo.server```
* ```rosrun wm_openpifpaf_ros_wrapper test_pose.py```
* ```rosrun image_view image_view image:=/pose_detection/image```
## Test
* cd in the openpifpafwebdemo
* ```curl -X POST -H "Content-Type: application/json" --data @test_image3.json http://localhost:5000/process```
* This will return the pose coordinates based on the image
