# Collect gestures data package

## Description

> The ROS package record data from 2 IMU MetaMotionR+ from Metawear ([Hardware datasheet](https://mbientlab.com/documents/MetaMotionR-PS3.pdf), [Software documentation](https://mbientlab.com/cppdocs/latest/index.html)). Each data is saved in a separated file. These data would later be used to train a classification model. This program output uniformised data, once calibrated.
> It also include the node *ros_metawear_simple* which publishes the data of the sensor fusion from the IMU. 
> Finally, in the notebook directory you can find the notebook used to process the data and train the model. 


## Installation 

* This program run in python 2.7.
* Clone this repository inside your desired catkin_ws/src then build it.

### Dependencies

* Python packages: Numpy, PyKDL, XLib.
* pyxhook is already provided in the folder.
* ROS: Already included messages.
* For the streaming node, one need also: mbientlab.metawear package

## Utilisation

> Command to launch the collecting node: *rosrun collect_gesture_data collect_data.py _address_imu_r:=E5:4D:16:18:CD:90 _address_imu_l:=F1:38:44:7A:F9:99 _gesture:=aggregate _calibrate:=False _session:=L1*

* *_gesture* is the name/label of the performed gesture.
* *_calibrate* is a boolean to say if one want to calibrate the sensor. It means that it align the axis with X-front, Y-left and Z-top. Must be down at the beginning of each session.
* *_session* is a string giving the name of the session. If *_calibration* is true, calibrate then save the parameters with this name. Otherwise take the saved parameters under this name.

> Command to launch the streaming node: *rosrun collect_gesture_data ros_metawear_simple.py _address:=E5:4D:16:18:CD:90*

* *_address* is the MAC address of the IMU.

### It subscribes to: 

* */metawear_ros_E5_4D_16_18_CD_90/rotation*: gives orientation of IMU (quaternion).
* */metawear_ros_E5_4D_16_18_CD_90/accel*: gives acceleration of IMU (g).
* */metawear_ros_E5_4D_16_18_CD_90/gyro*: gives angular speed of IMU (rad/s).

And that for both IMU. 
### Instruction for use

* (Optionnal) Calibrate sensor.
* (Optionnal) Change the directory where data are saved.
* Double click on your mouse to start/stop the recording of the gesture.
* The data from the different topics are synchronized based on their time stamp.
* The out put is a csv file with linear acceleration ( no gravity), rotation in quaternion and the angular velocity around the corrected axis.  
