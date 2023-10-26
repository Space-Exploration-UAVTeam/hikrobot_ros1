# hikrobot_ros1
ROS1 driver for hikrobot Aera scan cameras: https://www.hikrobotics.com/en/machinevision/visionproduct?typeId=78&id=145

## 1. Prerequisites
### 1.1 hikrobot MVS driver for Linux
https://www.hikrobotics.com/en/machinevision/service/download?module=0

### 1.2 ROS1
This package is tested under ROS Kinetic/Melodic/Noetic.

### 1.3 MAVROS 
We use the PX4 camera trigger function.
Install mavros https://github.com/mavlink/mavros to enable that.

## 2. Build 
Clone the repository to your catkin workspace (for example `~/catkin_ws/`):
```
cd ~/catkin_ws/src/
git clone https://github.com/Space-Exploration-UAVTeam/hikrobot_ros1.git
```
Then build the package with:
```
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
```
If you encounter any problem in prerequisites or building, we recommend you to try docker first.

## 3. Run
### 1.1 Single camera, with stand alone exposure time calculating
```
roslaunch hikrobot MvCameraPub.launch
```
### 1.2 Single camera, with camera AutoExposureTime mode(not suggested)
```
rosrun hikrobot MvCameraPub_Auto
```
### 1.3 Multiple cameras, with stand alone exposure time calculating
```
roslaunch hikrobot MvCameraPub_Multiple.launch
```
### 1.4 Multiple cameras, with stand alone exposure time calculating and hardware triggering
```
roslaunch hikrobot MvCameraPub_Trigger.launch
```

## 4. Hardware triggering
Following the PX4 camera trigger instruction https://docs.px4.io/v1.13/en/peripherals/camera.html
We here give a little more introduction:


## 5. Licence
The source code is released under BSD 3-Clause license.