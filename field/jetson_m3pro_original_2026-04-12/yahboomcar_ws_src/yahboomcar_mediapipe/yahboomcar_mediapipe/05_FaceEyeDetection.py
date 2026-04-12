#!/usr/bin/env python2
# encoding: utf-8
#import ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage,Image
#import define msg
from yahboomcar_msgs.msg import PointArray
#import commom lib
import cv2 as cv
import numpy as np
import time
import os

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
print("import done")
from arm_msgs.msg import ArmJoints
import cv2


class FaceEyeDetection(Node):
    def __init__(self,name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.eyeDetect = cv.CascadeClassifier( "/home/jetson/yahboomcar_ws/src/yahboomcar_mediapipe/yahboomcar_mediapipe/file/haarcascade_eye.xml")
        self.faceDetect = cv.CascadeClassifier(
"/home/jetson/yahboomcar_ws/src/yahboomcar_mediapipe/yahboomcar_mediapipe/file/haarcascade_eye.xml")
        self.pub_rgb = self.create_publisher(Image,"/FaceEyeDetection/image", 500)
		
        self.rgb_bridge = CvBridge()
        self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
        self.init_joints = [90, 150, 10, 20, 90, 90]
        while not self.TargetAngle_pub.get_subscription_count():
            self.pubSix_Arm(self.init_joints)
            time.sleep(0.1)
        self.pubSix_Arm(self.init_joints)
        self.pub_point = self.create_publisher(PointArray,'/mediapipe/points',1000)
        self.sub_rgb = self.create_subscription(Image,"/camera/color/image_raw",self.get_RGBImageCallBack,100)
        self.content_index = 0
        self.content = ["face", "eye", "face_eye"]

    def get_RGBImageCallBack(self,msg):
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, "bgr8")
        action = cv2.waitKey(1)
        if action == ord("f") or action == ord("F"):
            self.content_index += 1
            if self.content_index >= len(self.content): self.content_index = 0
        if self.content[self.content_index] == "face": frame = self.face(rgb_image)
        elif self.content[self.content_index] == "eye": frame = self.eye(rgb_image)
        else: frame = self.eye(self.face(rgb_image))			
        cv.imshow('frame', frame)
		


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


	

    def cancel(self):
        self.pub_rgb.unregister()

    def face(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        faces = self.faceDetect.detectMultiScale(gray, 1.3)
        for face in faces: frame = self.faceDraw(frame, face)
        return frame

    def eye(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        eyes = self.eyeDetect.detectMultiScale(gray, 1.3)
        for eye in eyes:
            cv.circle(frame, (int(eye[0] + eye[2] / 2), int(eye[1] + eye[3] / 2)), (int(eye[3] / 2)), (0, 0, 255), 2)
        return frame

    def faceDraw(self, frame, bbox, l=30, t=10):
        x, y, w, h = bbox
        x1, y1 = x + w, y + h
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
        # Top left x,y
        cv.line(frame, (x, y), (x + l, y), (255, 0, 255), t)
        cv.line(frame, (x, y), (x, y + l), (255, 0, 255), t)
        # Top right x1,y
        cv.line(frame, (x1, y), (x1 - l, y), (255, 0, 255), t)
        cv.line(frame, (x1, y), (x1, y + l), (255, 0, 255), t)
        # Bottom left x1,y1
        cv.line(frame, (x, y1), (x + l, y1), (255, 0, 255), t)
        cv.line(frame, (x, y1), (x, y1 - l), (255, 0, 255), t)
        # Bottom right x1,y1
        cv.line(frame, (x1, y1), (x1 - l, y1), (255, 0, 255), t)
        cv.line(frame, (x1, y1), (x1, y1 - l), (255, 0, 255), t)
        return frame

    def pub_img(self, frame):
        self.pub_rgb.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))


def main():
    rclpy.init()
    face_eye_detection = FaceEyeDetection('face_eye_detection')
    rclpy.spin(face_eye_detection)

