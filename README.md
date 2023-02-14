# Crying-MotorBike

Please download this!!

## 2023/01/16
1.成功下載Fusion 360

## 2023/01/17 
1.Fusion 360 to URDF

2.重灌ubuntu + roscore

3.import URDF to gazebo

4.更改transmittion、yaml

## 2023/02/11 

#### 1. gazebo launch:

    cd crying_MOTOROiD/
    source devel/setup.bash
    roslaunch Perfect_Wheel_description gazebo.launch
    
#### 2. controller launch:

    cd crying_MOTOROiD/
    source devel/setup.bash
    roslaunch Perfect_Wheel_description controller.launch
    
#### 3. imu:

    rostopic echo /imu
    
or
    
    cd crying_MOTOROiD/src/wheel/urdf
    python3 imu.py
    
#### 4. pid:
https://wiki.ros.org/rqt_reconfigure

    rosrun rqt_reconfigure rqt_reconfigure
    
#### 5. back wheel speed:
https://classic.gazebosim.org/tutorials?tut=ros_control&cat=connect_ros

    rostopic pub -1 /back_wheel_joint_position_controller/command std_msgs/Float64 "data: 1"
    
## 2023/02/15 

#### 1. 前叉角度傾斜20

#### 2. 車輪圓角加大

#### 3. 加配重

#### 4. 更改材質，輸入轉動慣量

    *問題：轉動慣量的計算原點和軸*







    
