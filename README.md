# Crying-MotorBike

Please download this!!

    sudo apt-get install ros*controller

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

    rostopic pub -1 /back_wh_position_controller/command std_msgs/Float64 "data: -1"
    rostopic pub -1 /front_wh_position_controller/command std_msgs/Float64 "data: -1"

    
## 2023/02/15 

#### 1. 前叉角度傾斜20

#### 2. 車輪圓角加大

#### 3. 加配重

#### 4. 更改材質，輸入轉動慣量

*問題：轉動慣量的計算原點和軸*


## 2023/02/28

#### 1. 更新轉動慣量

## 2023/03/15

#### 1. 前叉、後叉、body的材料是鋁
#### 2. 配重塊是stainless steel
#### 3. 輪子是rubber
#### 4. 本日最終版本：bibi(圖檔) bi(launch)

## 2023/04/21
#### 1. GA optimize PID

    roslaunch robot_launch launch_simulation.launch 
    
#### 2. test PID

    roslaunch robot_launch test_run.launch 
    
#### 3. plot u

    rqt_plot /back_fork_position_controller/command  

#### 4. plot theta   

    rqt_plot /bikeAng

## 2023/06/29
#### 1. test prbs

    roslaunch robot_launch test_prbs.launch 

#### 2. prbs config

    rostopic pub -1 /startornot std_msgs/Float64 "data: 1"
    rostopic pub -1 /refreshRate std_msgs/Float64 "data: 1"
    rostopic pub -1 /period std_msgs/Float64 "data: 0.001"

#### 3. rosbag record

    rosbag record /inout
    
#### 4. rosbag_to_csv

    rosdep install -r --ignore-src --from-paths src
    catkin_make
    source devel/setup.bash
    rosrun rosbag_to_csv rosbag_to_csv.py
    

https://answers.gazebosim.org/question/27178/looking-for-a-stable-solution-to-reset-a-robot-with-controller-in-gazebo/

https://www.instructables.com/Create-Internal-Interrupt-In-Arduino/

## 2023/07/13

    dmesg | grep tty
    sudo chmod 777 /dev/ttyACM0
    rosrun rosserial_python serial_node.py _baud:=115200 _port:=/dev/ttyACM0

https://github.com/Scottapotamas/xsens-mti

## 2023/07/18
Revolute 7 : Back Fork

Revolute 17: Front Fork

Revolute 20: Front Wheel

Revolute 25: Back Wheel

## 2023/10/24
My Beloved：

    Kp_p：6.8
    Kd_p：8.7

## 2023/11/26
### With battery

    Kp_p = 8
    Kd_p = 50
    dLimit = 8
    Kp_e = 19
    Ki_e = 0.05
    Kd_e = 1.35
    rate = 0.09

### Without battery

    Kp_p = 7
    Kd_p = 30
    dLimit = 7
    Kp_e = 25
    Ki_e = 0.085
    Kd_e = 1.4
    rate = 0.09
    
