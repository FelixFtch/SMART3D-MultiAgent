#!/usr/bin/env python3
# encoding: utf-8
import math
import time
import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from arm_msgs.msg import ArmJoints,ArmJoint
import cv2
from M3Pro_demo.media_library import *
import threading
print("import done")

class Hand(Node):
    def __init__(self,name):
        super().__init__(name)
		
        self.drawing = mp.solutions.drawing_utils
        self.timer = time.time()
        self.move_state = False
        self.points = []
        self.start_count = 0
        self.no_finger_timestamp = time.time()
        self.gc_stamp = time.time()
        self.hand_detector = HandDetector()


        self.pTime = 0

        # 定义抓取方块的状态
        self.one_grabbed = 0
        self.two_grabbed = 0
        self.three_grabbed = 0
        self.four_grabbed = 0

        self.block_num = 0

        # 定义手势识别次数
        self.Count_One = 0
        self.Count_Two = 0
        self.Count_Three = 0
        self.Count_Four = 0
        self.Count_Five = 0
		
        self.rgb_bridge = CvBridge()
        self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
        self.pub_SingleTargetAngle = self.create_publisher(ArmJoint, "arm_joint", 10)
        self.init_joints = [90, 164, 18, 0, 90, 30]
        while not self.TargetAngle_pub.get_subscription_count():
            self.pubSix_Arm(self.init_joints)
            time.sleep(0.1)
        self.pubSix_Arm(self.init_joints)
        self.sub_rgb = self.create_subscription(Image,"/camera/color/image_raw",self.get_RGBImageCallBack,100)



    def pubSingleArm(self, joint_id,joint_angle,run_time=2000):
        arm_joint =ArmJoint()
        arm_joint.joint = joint_angle
        arm_joint.id = joint_id
        arm_joint.time = run_time
        self.pub_SingleTargetAngle.publish(arm_joint)

	
    def get_RGBImageCallBack(self,msg):
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, "bgr8")
        frame, lmList,_ = self.hand_detector.findHands(rgb_image)
        #print("lmList: ",lmList)
        if len(lmList) != 0:
            gesture = self.hand_detector.get_gesture(lmList)
            #print("gesture = {}".format(gesture))
            
            if gesture == 'Yes':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_One = self.Count_One + 1
                self.Count_Two = 0
                self.Count_Three = 0
                self.Count_Four = 0
                self.Count_Five = 0
                if self.Count_One >= 5 and self.move_state == False:
                    self.move_state = True
                    self.Count_One = 0
                    print("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()


            elif gesture == 'OK':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_Five = self.Count_Five + 1
                self.Count_One = 0
                self.Count_Two = 0
                self.Count_Three = 0
                self.Count_Four = 0
                if self.Count_Five >= 5 and self.move_state == False:
                    self.move_state = True
                    self.Count_Five = 0
                    print("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()
        key = cv2.waitKey(1)			
        cv.imshow('frame', frame)

	

    def arm_ctrl_threading(self, gesture):
        if gesture == 'OK':
            move_joints = [163, 111, 0, 53, 90, 135]
            self.pubSix_Arm(move_joints)
            time.sleep(2.0)
            self.pubSingleArm(6,30)
            time.sleep(2.0)
            move_joints = [90, 164, 18, 0, 90, 135]
            self.pubSix_Arm(move_joints)
            time.sleep(2.0)

        elif gesture == 'Yes':
            move_joints = [90, 15, 65, 20, 90, 30]
            # Release clamping jaws 松开夹爪
            self.pubSingleArm(6,30)			
            time.sleep(2.0)
            # Move to object position 移动至物体位置
            self.pubSix_Arm(move_joints)			
            time.sleep(2.0)
            # Grasp and clamp the clamping claw进行抓取,夹紧夹爪
            self.pubSingleArm(6,135)
            time.sleep(2.0)
            move_joints = [90, 164, 18, 0, 90, 135]
            self.pubSix_Arm(move_joints)
            time.sleep(2.0)
        self.move_state = False

	

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


def main():
    print("start it")
    rclpy.init()
    hand_ = Hand("hand_detector")
    rclpy.spin(hand_)

