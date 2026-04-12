#!/usr/bin/env python3
# encoding: utf-8

#import ros lib
import rclpy
from rclpy.node import Node
from M3Pro_demo.media_library import *
import cv2 as cv
import numpy as np
import time
import os
import threading
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from arm_msgs.msg import ArmJoints
import cv2
print("import done")

class Hand(Node):
    def __init__(self,name, mode=False, maxHands=2, detectorCon=0.5, trackCon=0.5):
        super().__init__(name)
        self.hand_detector = HandDetector()
        #create a publisher
        self.rgb_bridge = CvBridge()
        self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
        self.init_joints = [90, 150, 10, 20, 90, 90]
        while not self.TargetAngle_pub.get_subscription_count():
            self.pubSix_Arm(self.init_joints)
            time.sleep(0.1)
        self.pubSix_Arm(self.init_joints)
        self.sub_rgb = self.create_subscription(Image,"/camera/color/image_raw",self.get_RGBImageCallBack,100)



    def get_RGBImageCallBack(self,msg):
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = self.process(rgb_image)
        key = cv2.waitKey(1)
        cv.imshow('dist', frame)
		


    def pubSix_Arm(self, joints, id=6, angle=180.0, runtime=2000):
        arm_joint =ArmJoints()
        arm_joint.joint1 = joints[0]
        arm_joint.joint2 = joints[1]
        arm_joint.joint3 = joints[2]
        arm_joint.joint4 = joints[3]
        arm_joint.joint5 = joints[4]
        arm_joint.joint6 = joints[5]
        arm_joint.time = runtime
        self.TargetAngle_pub.publish(arm_joint)
		
        
    def process(self, frame):
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        self.hand_detector.draw = True
        if len(lmList) != 0:
            hand = self.hand_detector.fingersUp(lmList)
        indexX = (bbox[0] + bbox[2]) / 2
        indexY = (bbox[1] + bbox[3]) / 2
        print("index X: %.1f, Y: %.1f" % (indexX, indexY))
        return frame


def main():
    print("start it")
    rclpy.init()
    hand_ = Hand('hand_node')
    rclpy.spin(hand_)


