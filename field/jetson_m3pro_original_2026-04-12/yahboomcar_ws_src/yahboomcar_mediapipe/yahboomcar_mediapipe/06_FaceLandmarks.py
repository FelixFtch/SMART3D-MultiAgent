#!/usr/bin/env python3

# encoding: utf-8

import time

import dlib

import cv2 as cv

import numpy as np

import rclpy

from rclpy.node import Node

import mediapipe as mp

from cv_bridge import CvBridge

from sensor_msgs.msg import Image

from arm_msgs.msg import ArmJoints

import cv2

print("import done")


class FaceLandmarks(Node):

    def __init__(self, name):
        super().__init__(name)

        self.dat_file = "/home/jetson/yahboomcar_ws/src/yahboomcar_mediapipe/yahboomcar_mediapipe/file/shape_predictor_68_face_landmarks.dat"

        self.hog_face_detector = dlib.get_frontal_face_detector()

        self.dlib_facelandmark = dlib.shape_predictor(self.dat_file)

        self.rgb_bridge = CvBridge()

        self.TargetAngle_pub = self.create_publisher(
            ArmJoints, "arm6_joints", 10)

        self.init_joints = [90, 150, 10, 20, 90, 90]

        while not self.TargetAngle_pub.get_subscription_count():

            self.pubSix_Arm(self.init_joints)

            time.sleep(0.1)

        self.pubSix_Arm(self.init_joints)

        self.sub_rgb = self.create_subscription(
            Image, "/camera/color/image_raw", self.get_RGBImageCallBack, 100)

    def get_RGBImageCallBack(self, msg):
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, "bgr8")

        frame = self.get_face(rgb_image, draw=False)

        frame = self.prettify_face(
            frame, eye=True, lips=True, eyebrow=True, draw=True)

        key = cv2.waitKey(1)

        cv.imshow('frame', frame)

    def pubSix_Arm(self, joints, id=6, angle=180.0, runtime=2000):

        arm_joint = ArmJoints()

        arm_joint.joint1 = joints[0]

        arm_joint.joint2 = joints[1]

        arm_joint.joint3 = joints[2]

        arm_joint.joint4 = joints[3]

        arm_joint.joint5 = joints[4]

        arm_joint.joint6 = joints[5]

        arm_joint.time = runtime

        self.TargetAngle_pub.publish(arm_joint)

    def get_face(self, frame, draw=True):

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        self.faces = self.hog_face_detector(gray)

        for face in self.faces:

            self.face_landmarks = self.dlib_facelandmark(gray, face)

            if draw:

                for n in range(68):

                    x = self.face_landmarks.part(n).x

                    y = self.face_landmarks.part(n).y

                    cv.circle(frame, (x, y), 2, (0, 255, 255), 2)

                    cv.putText(frame, str(n), (x, y),
                               cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        return frame

    def get_lmList(self, frame, p1, p2, draw=True):

        lmList = []

        if len(self.faces) != 0:

            for n in range(p1, p2):

                x = self.face_landmarks.part(n).x

                y = self.face_landmarks.part(n).y

                lmList.append([x, y])

                if draw:

                    next_point = n + 1

                    if n == p2 - 1:
                        next_point = p1

                    x2 = self.face_landmarks.part(next_point).x

                    y2 = self.face_landmarks.part(next_point).y

                    cv.line(frame, (x, y), (x2, y2), (0, 255, 0), 1)

        return lmList

    def get_lipList(self, frame, lipIndexlist, draw=True):

        lmList = []

        if len(self.faces) != 0:

            for n in range(len(lipIndexlist)):

                x = self.face_landmarks.part(lipIndexlist[n]).x

                y = self.face_landmarks.part(lipIndexlist[n]).y

                lmList.append([x, y])

                if draw:

                    next_point = n + 1

                    if n == len(lipIndexlist) - 1:
                        next_point = 0

                    x2 = self.face_landmarks.part(lipIndexlist[next_point]).x

                    y2 = self.face_landmarks.part(lipIndexlist[next_point]).y

                    cv.line(frame, (x, y), (x2, y2), (0, 255, 0), 1)

        return lmList

    def prettify_face(self, frame, eye=True, lips=True, eyebrow=True, draw=True):

        if eye:

            leftEye = self.get_lmList(frame, 36, 42)

            rightEye = self.get_lmList(frame, 42, 48)

            if draw:

                if len(leftEye) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(leftEye), (0, 0, 0))

                if len(rightEye) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(rightEye), (0, 0, 0))

        if lips:

            lipIndexlistA = [51, 52, 53, 54, 64, 63, 62]

            lipIndexlistB = [48, 49, 50, 51, 62, 61, 60]

            lipsUpA = self.get_lipList(frame, lipIndexlistA, draw=True)

            lipsUpB = self.get_lipList(frame, lipIndexlistB, draw=True)

            lipIndexlistA = [57, 58, 59, 48, 67, 66]

            lipIndexlistB = [54, 55, 56, 57, 66, 65, 64]

            lipsDownA = self.get_lipList(frame, lipIndexlistA, draw=True)

            lipsDownB = self.get_lipList(frame, lipIndexlistB, draw=True)

            if draw:

                if len(lipsUpA) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(lipsUpA), (249, 0, 226))

                if len(lipsUpB) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(lipsUpB), (249, 0, 226))

                if len(lipsDownA) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(lipsDownA), (249, 0, 226))

                if len(lipsDownB) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(lipsDownB), (249, 0, 226))

        if eyebrow:

            lefteyebrow = self.get_lmList(frame, 17, 22)

            righteyebrow = self.get_lmList(frame, 22, 27)

            if draw:

                if len(lefteyebrow) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(lefteyebrow), (255, 255, 255))

                if len(righteyebrow) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(righteyebrow), (255, 255, 255))

        return frame


def main():

    print("start it")

    rclpy.init()

    landmarks = FaceLandmarks("face_landmarks_node")

    rclpy.spin(landmarks)

