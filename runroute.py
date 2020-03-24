#!/usr/bin/env python

'''
Copyright (c) 2016, Nadya Ampilogova
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''
import threading
import time
import copy
from Queue import Queue
from math import atan2

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import yaml

from gotopos import GoToPose
from rotate import RotateController, PI
from utils import distance, upload, radiou2dgree
import config

original_pose = None
cur_x, cur_y, cur_theta = 0, 0, 0
robot_status = {
    'id': config.RobotID,
    'route_point_no': None,
    'route_point_pos': (0, 0), #(x, y)
    'holding_pos': (0, 0), #(x, y)
    #for element in 'enter', 'stay', 'leave', it will be {angle: timestamp}
    #'enter' and leave' will have only 1 element
    #'stay' will have sevel elements
    'enter': {},
    'stay': [],             
    'leave': {}
    }
pose_queue = Queue(maxsize=0)
lock = threading.Lock()
running_flag = threading.Event()
running_flag.set()

def resetRbotStatus(waypoint_no=None):
    robot_status['route_point_no'] = waypoint_no
    robot_status['route_point_pos'] = (0, 0)
    robot_status['holding_pos'] = (0, 0)
    robot_status['enter'] = {}
    robot_status['stay'] = []
    robot_status['leave'] = {}

def readPose(msg):
    global original_pose

    if original_pose is None:
        original_pose = msg.pose.pose
        rospy.loginfo('readPose: find start pose: {}'.format(original_pose))
    pose_record = {
        'pose': msg.pose.pose,
        'time': time.time()   #TODO: check if there is time is msg
    }

    pose_queue.put(pose_record)

def analyzePose():
    global cur_x
    global cur_y
    global cur_theta

    while running_flag.isSet():
        if pose_queue is None:
            rospy.loginfo('analyzePose: main process exit! exit')
            return

        if pose_queue.empty():
            continue

        cur_time = time.time()

        pose_record = pose_queue.get()
        pose_pos = pose_record['pose']
        # pose_time = pose_record['time']

        cur_x = pose_pos.position.x
        cur_y = pose_pos.position.y

        rot_q = pose_pos.orientation
        (_,_,cur_theta) = euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        #convert form radius to degree
        cur_theta = radiou2dgree(cur_theta)

        # rospy.loginfo("current position: x-{}, y-{}, theta-{}".format(cur_x, cur_y, cur_theta))
        
        #robot not arrive at a point or already leave a point
        if robot_status['route_point_no'] is None or robot_status['leave_time']!=0:
            continue

        #tell if robot leave current point
        if (cur_time - robot_status['enter_time']) > (config.Holding_Time) or \
            distance(robot_status['holding_pos'], (cur_x, cur_y, cur_theta)) > config.Valid_Range_Radius:
            
            lock.acquire()
            robot_status['leave'] = (cur_theta, cur_time)
            rospy.loginfo('ananlyzePose: find leave waypoint time, the record of current waypoint is: \n {}'.format(robot_status))

            #send out the status record and reset it
            t = threading.Thread(target=upload, args=(robot_status,))
            t.start()

            resetRbotStatus()

            lock.release()

            continue

    ##  for only record the event of robot leaving a waypoint, these angle record will be sent to TSDB
    ##  by a time based regular process, so we disabled the following codes
    #now, the robot should be staying at the point, we record its different angle position
    # if len(robot_status['stay']) > (config.Circle_Rotate_Steps-1):
    #     #no need to reocord the last rotation restuls, becuase it will be same as robot_status['leave']]
    #     continue
    # pre_direction = robot_status['enter'][0] if len(robot_status['stay'])==0 else robot_status['stay'][-1][0]
    # angle_var = radiou2dgree(abs(cur_theta-pre_direction))
    # if angle_var > (360/config.Circle_Rotate_Steps):
    #     lock.acquire()
    #     robot_status['stay'].append((cur_theta, cur_time)) 
    #     lock.release()

def buildFullRoute(route, original_pose):
    #prepare navigation route to make robot return to original position after the job
    ##add reversed point list and original robot pos into the route
    full_route = copy.deepcopy(route)
    route_len = len(route)
    return_index = range(2, route_len+1)
    return_index.reverse()
    for pt, index in zip(route[:-1][::-1], return_index):
        pt['point_no'] = index*-1
        full_route.append(pt)
    pt = copy.deepcopy(route[0])
    pt['point_no'] = -1
    if original_pose is None:
        pt['position']['x'], pt['position']['y'] = 0, 0
        pt['quaternion']['r1'], pt['quaternion']['r2'], \
            pt['quaternion']['r3'],  pt['quaternion']['r4'] = 0, 0, 0, 1
    else:
        #TODO: for multirobots,
        # it will be better to obtain robot's original pos from server at the beginning
        pt['position']['x'], pt['position']['y'] = original_pose.position.x, original_pose.position.y
        pt['quaternion']['r1'], pt['quaternion']['r2'], \
            pt['quaternion']['r3'],  pt['quaternion']['r4'] = original_pose.orientation.x, \
                                                                  original_pose.orientation.y, \
                                                                  original_pose.orientation.z, \
                                                                  original_pose.orientation.w
    full_route.append(pt)
    rospy.loginfo('runRoute: full route: \n {}'.format(full_route))

    return full_route
    
def runRoute(route):

    if type(route) != list:
        raise TypeError('runRoute() required param route in type: list')

    if len(route) == 0:
        rospy.loginfo('runRoute: route point list is empty, return!')
    try:
        # Initialize
        rospy.init_node('follow_route', anonymous=False)

        # start to probe robot's position
        odom_sub = rospy.Subscriber("/odom", Odometry, readPose)
        t = threading.Thread(target=analyzePose, args=())
        t.start()

        #init the rotate controller
        rotate_ctl =  RotateController()
        

        #build the full route to make the robot return to its original position
        full_route = buildFullRoute(route, original_pose)
        route_len = len(route)
        
        #start navigation
        navigator = GoToPose()
        for index, pt in enumerate(full_route, start=1):

            if rospy.is_shutdown():
                running_flag.clear()
                break

            pt_num = pt['point_no']

            # Navigation
            rospy.loginfo("Go to No. {} pose".format(pt_num))
            success = navigator.goto(pt['position'], pt['quaternion'])
            if not success:
                rospy.loginfo("Failed to reach No. {} pose".format(pt_num))
                continue
            rospy.loginfo("Reached No. {} pose".format(pt_num))

            if index > route_len or robot_status['route_point_no'] is not None:
                # returning route
                # or the route point was reached already
                continue    
 
            #write point enter information
            writeEnterEvent(pt_num, pt)

            #commend to robot to rotate 360 degree at current place
            step_angle = 360*1.0 / config.Circle_Rotate_Steps
            for i in range(1, config.Circle_Rotate_Steps+1):
                rospy.loginfo('runRoute: rotate step {}, rotate angle: {}'.format(i, step_angle))
                rotate_ctl.rotate(angle=step_angle, speed=config.Rotate_Speed)
                rospy.sleep(config.Holding_Step_Time/config.Circle_Rotate_Steps)


            #this guarantee to send the parameters out
            rospy.sleep(0.5)

        #to make the analyzePose thread finished after unsubscribe the odom topic
        odom_sub.unregister()
        running_flag.clear()
        pose_queue = None
        rospy.loginfo('runRoute: finished route, unregister topic odom!')

    except rospy.ROSInterruptException:
        running_flag.clear()
        pose_queue = None
        odom_sub.unregister()
        rospy.loginfo("Ctrl-C caught. Quitting")

def writeEnterEvent(pt_num, pt):
    lock.acquire()
    robot_status['route_point_no'] = pt_num
    robot_status['enter'] = (cur_theta, time.time())
    robot_status['route_point_pos'] = (pt['position']['x'], pt['position']['y'])
    robot_status['holding_pos'] = (cur_x, cur_y)
    rospy.loginfo('runRoute: arrive at a waypoint,  the record of current waypoint is: \n {}'.format(robot_status))
    lock.release()

if __name__ == '__main__':
        # Read information from yaml file
    with open("route.yaml", 'r') as stream:
        dataMap = yaml.load(stream)

    runRoute(dataMap)
