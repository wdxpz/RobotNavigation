# Keynotes 
## 网络时间协议(NTP，Network Time Protocol)
* 设置方法是安装chrony之后用ntpdate命令指定ntp服务器即 可。
* 这样一来会表示服务器和当前计算机之间的时间误差，进而会调到服务器的时间。这 就是通过给不同的PC指定相同的NTP服务器，将时间误差缩短到最小的方法。

```
$ sudo apt-get install -y chrony ntpdate 
$ sudo ntpdate -q ntp.ubuntu.com
```

# ROS Knowledge 
1. [ROS python client library](http://wiki.ros.org/rospy)
2. [ROS Command](http://wiki.ros.org /ROS/CommandLineTools), [cheatsheet](https://github.com/ros/cheatsheet/releases)

# Operation
## SLAM
1. in remote-pc, open new terminal, run

```
$ roscore
```
2. ssh Turtlebot pc, in terminal, run 

```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
you will see:

```
[INFO] [1583226443.156238]: --------------------------
[INFO] [1583226443.159682]: Connected to OpenCR board!
[INFO] [1583226443.163006]: This core(v1.2.3) is compatible with TB3 Waffle or Waffle Pi
[INFO] [1583226443.166428]: --------------------------
[INFO] [1583226443.169656]: Start Calibration of Gyro
[INFO] [1583226445.526265]: Calibration End
```

1. in remote-pc, open new terminal, run

```
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
to do visual slam

1. in remote-pc, run 

```
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
you will see

```
Control Your TurtleBot3!
  ---------------------------
  Moving around:
          w
     a    s    d
          x

  w/x : increase/decrease linear velocity
  a/d : increase/decrease angular velocity
  space key, s : force stop

  CTRL-C to quit
```
to teleoperate the robot to move

1. save the map, in remote-pc, run 

```
$ rosrun map_server map_saver -f /save_dir
```

## navigation
1. remote pc:

```
$ roscore
```
1. turtulebot pc:

```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
1. remote pc:

```
#The ${TB3_MODEL} is the name of the model you are using in burger, waffle, waffle_pi
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```
1. Estimate Initial Pose in remote pc:
    * Click the 2D Pose Estimate button.
    * Click on the approxtimate point in the map where the TurtleBot3 is located and drag the cursor to indicate the direction where TurtleBot3 faces.
    * Then move the robot back and forth with tools like the turtlebot3_teleop_keyboard node to collect the surrounding environment information and find out where the robot is currently located on the map.
    * The turtlebot3_teleop_keyboard node used for Estimate Initial Pose should be terminated after use. If it does not, the robot will behave strangely because the topic overlaps with the /cmd_vel topic from the navigation node of the next step.

```
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

1. Send Navigation Goal at remote pc:
    * Click the 2D Nav Goal button, in the menu of RViz, a very large green arrow appears. This green arrow is a marker that can specify the destination of the robot. The root of the arrow is the x and y position of the robot, and the orientation pointed by the arrow is the theta direction of the robot
    * Click on a specific point in the map to set a goal position and drag the cursor to the direction where TurtleBot should be facing at the end

## Simulation - Gazebo
1. Gazebo Installation
    * refer [通过Gazebo仿真学TurtleBot3（二）——环境搭建](https://blog.csdn.net/u010853356/article/details/79226764)
    * install TB3 msg, funciton, simulation packages
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws
$ rosdep install --from-paths src -i -y
$ catkin_make
```
注意以上命令第7行，采用rosdep install依赖安装的方式，安装了TB3代码中依赖的各种ROS软件包。rosdep install主要基于软件package包目录下package.xml文件中的依赖项关系来安装依赖项，具体说明见ROS官方wiki。
catkin_make编译后，还需要将新工程的ROS环境设置加入~/.bashrc文件，命令如下：

```
$ echo "source ~/catkin_ws/devel/setup.bash " >> ~/.bashrc
$ source ~/.bashrc
```
最后还可以根据所选用的TB3机器人是Burger还是Waffle，将其作为环境变量加入~/.bashrc文件，以方便后续使用。否则每次运行程序都需要先输入“export TURTLEBOT3_MODEL=burger”或者“export TURTLEBOT3_MODEL=waffle”。
　　命令如下：

```
$ echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
$ source ~/.bashrc
```
2. 


# Development
## Hosts
1. Turtlebot remote-pc:
host: ubuntu16 on virtualbox
ip:192.168.3.89
account:sw
password:abc123!@#

2. Turtlebot pc:
host: raspberry pi 3 on bot
ip: 192.168.3.90
account: waffle
password: 123456

## how to get robot position and angle from the original position
1. subscribe /odom topic to get robot pose (position and orientation), refer [How to know the Pose of a robot (Python) ?](https://www.theconstructsim.com/ros-qa-know-pose-robot-python/)
   
```
Now we will create a script named inside the check_odometry/src/ directory. Add the following code to the script file
#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    print(msg.pose.pose)
    
rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()

#We will now create a launch file with name inside the check_odometry/launch/ directory with the following content
<launch>
    <node pkg="check_odometry" type="check_odom.py" name="check_odometry" output="screen" />
</launch>

```
1. convert orientation to angle, refer [How to know the direction that the Robot is pointing to from pose?](https://answers.ros.org/question/196938/how-to-know-the-direction-that-the-robot-is-pointing-to-from-pose/)

```
#C++
tf::Quaterion quat;
tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);
tf::Matrix3x3 orTmp(quat);
orTmp.getRPY(roll, pitch, yaw);

#python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
x = 0.0
y= 0.0
theta = 0.0
def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll,pitch,theta) = euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
rospy.init_node("speed_controller")
sub = rospy.Subscriber("/odom",Odometry,newOdom)
```
## save ROS map to readable image
1. refer to [how to correctly convert OccupancyGrid format message to image ?](https://answers.ros.org/question/163801/how-to-correctly-convert-occupancygrid-format-message-to-image/)
```
def callback(self,data):
     self.width = data.info.width
     self.height = data.info.height
     self.resolution = data.info.resolution
     self.length = len(data.data)
     #self.min_line = []

     #creat an mat to load costmap
     costmap_mat = cv.CreateMat(self.height,self.width,cv.CV_8UC1)
     
     for i in range(1,self.height):
            for  j in range(1,self.width):
               cv.Set2D(costmap_mat,self.width-j,self.height-i,255-int(float(data.data[(i-1)*self.width+j])/100*255))
```
1. maybe change subscribe to topic `nav_msgs/OccupancyGrid`, see [map_server](http://wiki.ros.org/map_server)
2. or directly using `hector_compressed_map_transport`, [github](https://github.com/tu-darmstadt-ros-pkg/hector_slam/tree/catkin/hector_compressed_map_transport)

## autonomous move robot 
1. refer to [Learn TurtleBot and ROS](https://learn.turtlebot.com/), or find the source codes in [turtlebot](/docs/turtlebot)
    * [move to a specific point in the map](https://learn.turtlebot.com/2015/02/01/14/), see [code](https://github.com/markwsilliman/turtlebot/blob/master/go_to_specific_point_on_map.py)
    * [move the robot along a route, and execute task at each point](https://learn.turtlebot.com/2015/02/04/5/), see [code](https://github.com/markwsilliman/turtlebot/blob/master/follow_the_route.py)
    * [get the battery status](https://learn.turtlebot.com/2015/02/01/16/), see [code](https://github.com/markwsilliman/turtlebot/blob/master/kobuki_battery.py)
