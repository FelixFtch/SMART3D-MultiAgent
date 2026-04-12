#!/usr/bin/env python3
# encoding: utf-8
import math
import time
import cv2 as cv
import numpy as np
import mediapipe as mp
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from arm_msgs.msg import ArmJoints
import cv2
print("import done")

class handDetector(Node):
    def __init__(self,name):
        super().__init__(name)
        self.xp = self.yp = self.pTime = self.boxx = 0
        self.tipIds = [4, 8, 12, 16, 20]
        self.imgCanvas = np.zeros((480, 640, 3), np.uint8)
        self.brushThickness = 5
        self.eraserThickness = 100
        self.top_height = 50
        self.Color = "Red"
        self.ColorList = {
		    'Red': (0, 0, 255),
		    'Green': (0, 255, 0),
		    'Blue': (255, 0, 0),
		    'Yellow': (0, 255, 255),
		    'Black': (0, 0, 0),
			}
        self.tipIds = [4, 8, 12, 16, 20]
        self.mpHand = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.85,
            min_tracking_confidence=0.5 )
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=15)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=10, circle_radius=10)

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
        h, w, c = rgb_image.shape
        frame,lmList = self.findHands(rgb_image, draw=False)
        key = cv2.waitKey(1)
        if len(lmList) != 0:
            # print(lmList)
            # tip of index and middle fingers
            x1, y1 = lmList[8][1:]
            x2, y2 = lmList[12][1:]
            fingers = self.fingersUp()
            if fingers[1] and fingers[2]:
                # print("Seclection mode")
                if y1 < self.top_height:
                    if 0 < x1 < int(w / 5) - 1:
                        self.boxx = 0
                        self.Color = "Red"
                    if int(w / 5) < x1 < int(w * 2 / 5) - 1:
                        self.boxx = int(w / 5)
                        self.Color = "Green"
                    elif int(w * 2 / 5) < x1 < int(w * 3 / 5) - 1:
                        self.boxx = int(w * 2 / 5)
                        self.Color = "Blue"
                    elif int(w * 3 / 5) < x1 < int(w * 4 / 5) - 1:
                        self.boxx = int(w * 3 / 5)
                        self.Color = "Yellow"
                    elif int(w * 4 / 5) < x1 < w - 1:
                        self.boxx = int(w * 4 / 5)
                        self.Color = "Black"
                cv.rectangle(frame, (x1, y1 - 25), (x2, y2 + 25), self.ColorList[self.Color], cv.FILLED)
                cv.rectangle(frame, (self.boxx, 0), (self.boxx + int(w / 5), self.top_height), self.ColorList[self.Color], cv.FILLED)
                cv.rectangle(frame, (0, 0), (int(w / 5) - 1, self.top_height), self.ColorList['Red'], 3)
                cv.rectangle(frame, (int(w / 5) + 2, 0), (int(w * 2 / 5) - 1, self.top_height), self.ColorList['Green'], 3)
                cv.rectangle(frame, (int(w * 2 / 5) + 2, 0), (int(w * 3 / 5) - 1, self.top_height), self.ColorList['Blue'], 3)
                cv.rectangle(frame, (int(w * 3 / 5) + 2, 0), (int(w * 4 / 5) - 1, self.top_height), self.ColorList['Yellow'], 3)
                cv.rectangle(frame, (int(w * 4 / 5) + 2, 0), (w - 1, self.top_height), self.ColorList['Black'], 3)
            if fingers[1] and fingers[2] == False and math.hypot(x2 - x1, y2 - y1) > 50:
                # print("Drawing mode")
                if self.xp == self.yp == 0: self.xp, self.yp = x1, y1
                if self.Color == 'Black':
                    cv.line(frame, (self.xp, self.yp), (x1, y1), self.ColorList[self.Color], self.eraserThickness)
                    cv.line(self.imgCanvas, (self.xp, self.yp), (x1, y1), self.ColorList[self.Color], self.eraserThickness)
                else:
                    cv.line(frame, (self.xp, self.yp), (x1, y1), self.ColorList[self.Color], self.brushThickness)
                    cv.line(self.imgCanvas, (self.xp, self.yp), (x1, y1), self.ColorList[self.Color], self.brushThickness)
                cv.circle(frame, (x1, y1), 15, self.ColorList[self.Color], cv.FILLED)
                self.xp, self.yp = x1, y1
            else: self.xp = self.yp = 0
        imgGray = cv.cvtColor(self.imgCanvas, cv.COLOR_BGR2GRAY)
        _, imgInv = cv.threshold(imgGray, 50, 255, cv.THRESH_BINARY_INV)
        imgInv = cv.cvtColor(imgInv, cv.COLOR_GRAY2BGR)
        frame = cv.bitwise_and(frame, imgInv)
        frame = cv.bitwise_or(frame, self.imgCanvas)
        cv.rectangle(frame, (20, h - 100), (50, h - 70), self.ColorList[self.Color], cv.FILLED)
        cv.putText(frame, self.Color, (70, h - 75), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
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


    def findHands(self, frame, draw=True):
        self.lmList = []
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw: self.mpDraw.draw_landmarks(frame, handLms, self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
                else: self.mpDraw.draw_landmarks(frame, handLms, self.mpHand.HAND_CONNECTIONS)
            for id, lm in enumerate(self.results.multi_hand_landmarks[0].landmark):
                h, w, c = frame.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                # print(id, cx, cy)
                self.lmList.append([id, cx, cy])
        return frame, self.lmList

    def fingersUp(self):
        fingers=[]
        # Thumb
        if (self.calc_angle(self.tipIds[0],
                            self.tipIds[0] - 1,
                            self.tipIds[0] - 2) > 150.0) and (
                self.calc_angle(
                    self.tipIds[0] - 1,
                    self.tipIds[0] - 2,
                    self.tipIds[0] - 3) > 150.0): fingers.append(1)
        else:            fingers.append(0)
        # 4 finger
        for id in range(1, 5):
            if self.lmList[self.tipIds[id]][2] < self.lmList[self.tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)
        return fingers



    def get_dist(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return abs(math.sqrt(math.pow(abs(y1 - y2), 2) + math.pow(abs(x1 - x2), 2)))

    def calc_angle(self, pt1, pt2, pt3):
        point1 = self.lmList[pt1][1], self.lmList[pt1][2]
        point2 = self.lmList[pt2][1], self.lmList[pt2][2]
        point3 = self.lmList[pt3][1], self.lmList[pt3][2]
        a = self.get_dist(point1, point2)
        b = self.get_dist(point2, point3)
        c = self.get_dist(point1, point3)
        try:
            radian = math.acos((math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2)) / (2 * a * b))
            angle = radian / math.pi * 180
        except:
            angle = 0
        return abs(angle)



def main():
    print("start it")
    rclpy.init()
    hand_detector = handDetector("hand_detector")
    rclpy.spin(hand_detector)

