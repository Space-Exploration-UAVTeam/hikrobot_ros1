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
roslaunch mavros px4.launch
roslaunch hikrobot MvCameraPub_Trigger.launch
```

## 4. Hardware triggering
Following the PX4 camera trigger instruction https://docs.px4.io/v1.13/en/peripherals/camera.html  
We here give a little more introduction:  
 <img src="https://github.com/Space-Exploration-UAVTeam/hikrobot_ros1/blob/master/img/Picture1.png" width="600" />  
The picture above show how this works with the PX4/Pixhawk Flight Control Unit through mavros. After our camera node send a trigger_enable request, the FCU send the trigger signal to the camera and the time stamp of that signal to the computer. Image data will come to the computer much slower than the time stamp, so we create a buffer for the time stamp and some buffers for the image data. Whenever all buffers contain the new elements, we publish them together.  
The time delays in the picture is just an example...  
Set the FCU parameters in QGC:  
 <img src="https://github.com/Space-Exploration-UAVTeam/hikrobot_ros1/blob/master/img/Picture2.png" width="400" />
 <img src="https://github.com/Space-Exploration-UAVTeam/hikrobot_ros1/blob/master/img/Picture3.png" width="400" />  
Trigger mode: Time based, on commnad;  
AUX Pins are the output of the trigger signal to the camera；  
Both TRIG_ACT_TIME 与 TRIG_INTERVAL(camera frequency) are needed， where the former should be smaller than the later, of course.  
Reboot the FCU to active the parameters.  
Use A wire to connect a Signal pin of the AUX Pins and the trigger pin of hikrobot camera; If you have multiple cameras, we suggest you still connect the wires to A Signal pin, not multiple Signal pins.  
The camera must be grounded. If you use all these on a drone or other moving paltform, you should be careful about that.  

## 5. Licence
The source code is released under BSD 3-Clause license.
