#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
# @brief    [py example gripper] gripper test for doosan robot
# @author   Jin Hyuk Gong (jinhyuk.gong@doosan.com)

import rospy
import os
import threading, time
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py

# for single robot
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *


def robotiq_2f_open():
    pass

def robotiq_2f_close():
    pass

def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop.publish(stop_mode=1) #STOP_TYPE_QUICK)
    return 0

# convert list to Float64MultiArray
def _ros_listToFloat64MultiArray(list_src):
    _res = []
    for i in list_src:
        item = Float64MultiArray()
        item.data = i
        _res.append(item)
    return _res

if __name__ == "__main__":
    #----- set target robot ---------------
    my_robot_id    = "dsr01"
    my_robot_model = "m1013"
    SET_ROBOT(my_robot_id, my_robot_model)

    robot = moveit_commander.RobotCommander(robot_description="dsr01m1013/robot_description")

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('rsc_project')
    rospy.on_shutdown(shutdown)
    scene = moveit_commander.PlanningSceneInterface(ns="dsr01m1013")

    rospy.sleep(2)

    boxes = ['box1', 'box2', 'box3']
    positions = [[1, -0.2, 0], [1, 0, 0], [1, 0.2, 0]]

    for box in boxes:
        scene.remove_attached_object('link6', box)

    for index, box in enumerate(boxes):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = positions[index][0]
        box_pose.pose.position.y = positions[index][1]
        box_pose.pose.position.z = positions[index][2]
        box_name = box
        scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)

    srv_robotiq_2f_move = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/gripper/robotiq_2f_move', Robotiq2FMove)

    p0 = posj(0, 0, 0, 0, 0, 0)


    poss = [[posj(-13, 60, 60, 0, 60, 0), posj(2, -56, -64, 0, -59, 0)],
            [posj(-2, 60, 60, 0, 60, 0), posj(2, -51, -63, 0, -66, 0)],
            [posj(9, 60, 60, 0, 60, 0), posj(2, -47, -59, 0, -73, 0)]]

    while not rospy.is_shutdown():
        movej(p0, vel=60, acc=30)
        print("movej(p0)")
        wait(1)

        for idx, pos in enumerate(poss):
            movej(pos[0], vel=100, acc=30)
            print(f'pick {pos[0]}')
            scene.attach_box('link6', boxes[idx]);
            movej(pos[1], vel=100, acc=30)
            print(f'place {pos[1]}')
            scene.remove_attached_object('link6', boxes[idx])
            wait(1)


        movej(p0, vel=60, acc=30)
        print("movej(p0)")
        break

    print('good bye!')
