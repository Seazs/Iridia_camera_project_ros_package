# Iridia_camera_project_ros_package
ros package for door detection on rvr robot

## Installing the 
Do the following commande to clone the repo, compile the file and source the 
```
cd
git clone https://github.com/Seazs/Iridia_camera_project_ros_package.git
cd ./Iridia_camera_project_ros_package/
catkin_make
source ~/Iridia_camera_project_ros_package/devel/setup.bash
```
You can add the source commande directly to your .bashrc file to automatically have it sourced.

## Dependancies:


- **Ultralytics**:
```
pip install ultralytics
``` 
- **PyTorch**: refere to the torche installation guide found in this page:
https://docs.ultralytics.com/quickstart/
- **numpy** and **opencv**


## How to run it
Once camera is setup (You might need to change the index camera in the Camera_publisher script), do the following command in the workspace folder:
```
roslaunch ./src/up_pointing_camera/launch/camera.launch
```

## How to modify detection
If you want to update the weight file with a new one, simply change the door_weight.pt file with yours.



## How to use the result:
The distance in centimeters (dx, dy) are published on the *door_detection* topic with a **Int16MultiArray** message type.