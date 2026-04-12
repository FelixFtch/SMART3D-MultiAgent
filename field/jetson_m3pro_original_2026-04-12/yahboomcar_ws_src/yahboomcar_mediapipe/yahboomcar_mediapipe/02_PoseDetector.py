#!/usr/bin/env python3
# encoding: utf-8

#import ros lib
import rclpy
from rclpy.node import Node
import mediapipe as mp
import cv2 as cv
import numpy as np
import time
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from arm_msgs.msg import ArmJoints
import cv2
print("import done")

class PoseDetector(Node):
    def __init__(self, name,mode=False, smooth=True, detectionCon=0.5, trackCon=0.5):
        super().__init__(name)
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        self.pose = self.mpPose.Pose(
            static_image_mode=mode,
            smooth_landmarks=smooth,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackCon )
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)
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
        frame, img = self.pubPosePoint(rgb_image, draw=False)
        dist = self.frame_combine(frame, img)
        key = cv2.waitKey(1)
        cv.imshow('dist', dist)
		


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
	

    def pubPosePoint(self, frame, draw=True):
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(img_RGB)
        if self.results.pose_landmarks:
            if draw: self.mpDraw.draw_landmarks(frame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
        return frame, img

    def frame_combine(slef,frame, src):
        if len(frame.shape) == 3:
            frameH, frameW = frame.shape[:2]
            srcH, srcW = src.shape[:2]
            dst = np.zeros((max(frameH, srcH), frameW + srcW, 3), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        else:
            src = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
            frameH, frameW = frame.shape[:2]
            imgH, imgW = src.shape[:2]
            dst = np.zeros((frameH, frameW + imgW), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        return dst

def main():
    print("start it")
    rclpy.init()
    pose_detector = PoseDetector('pose_detector')
    rclpy.spin(pose_detector)

