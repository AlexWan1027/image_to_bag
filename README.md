# image to bag for kitti odometry datasets
## 1. Prerequisites
Ubuntu  16.04.   ROS Kinetic. OpenCV 3

## 2. Build the project on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/AlexWan1027/image_to_bag.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
## 3. set the paramters for project
```
     1) open the config file: /config/config.yaml
     2) use global file name to set the image path for left and right image
     3) set the time file
     4) set the bag name(save in the path: /result/ by default)
```

## 4. Run the project with launch file
```
     roslaunch image_to_bag image_to_bag.launch
```