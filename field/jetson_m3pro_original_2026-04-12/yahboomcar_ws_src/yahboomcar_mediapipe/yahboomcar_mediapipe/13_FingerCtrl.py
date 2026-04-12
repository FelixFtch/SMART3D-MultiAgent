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
from arm_msgs.msg import ArmJoints,ArmJoint
import cv2
print("import done")

pTime = cTime = volPer = value = index = 0
effect = ["color", "thresh", "blur", "hue", "enhance"]
volBar = 400
class handDetector(Node):
    def __init__(self, name):
        super().__init__(name)
        self.effect = ["color", "thresh", "blur", "hue", "enhance"]
        self.lmList = []
        self.volBar = 400
        self.pTime = self.cTime = self.volPer = self.value = self.index = 0		
        self.mpHand = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=15)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=10, circle_radius=10)


        self.rgb_bridge = CvBridge()
        self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
        self.pub_SingleTargetAngle = self.create_publisher(ArmJoint, "arm_joint", 10)
        self.init_joints = [90, 150, 10, 20, 90, 180]
        while not self.TargetAngle_pub.get_subscription_count():
            self.pubSix_Arm(self.init_joints)
            time.sleep(0.1)
        self.pubSix_Arm(self.init_joints)
        self.sub_rgb = self.create_subscription(Image,"/camera/color/image_raw",self.get_RGBImageCallBack,100)

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

    def pubSingleArm(self, joint_id,joint_angle,run_time=500):
        arm_joint =ArmJoint()
        arm_joint.joint = joint_angle
        arm_joint.id = joint_id
        arm_joint.time = run_time
        self.pub_SingleTargetAngle.publish(arm_joint)

    def get_RGBImageCallBack(self,msg):
        frame = self.rgb_bridge.imgmsg_to_cv2(msg, "bgr8")
        img = self.findHands(frame)
        lmList =  self.findPosition(frame, draw=False)
        if len(lmList) != 0:
            angle =  self.calc_angle(4, 0, 8)
            print("angle: ",angle)
            if angle<2:
                angle = 2
            grasp = 360/angle
            self.pubSingleArm(6,int(grasp))

        cv.imshow('dst', frame)
        action = cv2.waitKey(1)


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

    def findHands(self, frame, draw=True):
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)		
        return img

    def findPosition(self, frame, draw=True):
	    self.lmList = []
	    if self.results.multi_hand_landmarks:
	    	for id, lm in enumerate(self.results.multi_hand_landmarks[0].landmark):
	    		# print(id,lm)
	    		h, w, c = frame.shape
	    		cx, cy = int(lm.x * w), int(lm.y * h)
	    		# print(id, lm.x, lm.y, lm.z)
	    		self.lmList.append([id, cx, cy])
	    		if draw: cv.circle(frame, (cx, cy), 15, (0, 0, 255), cv.FILLED)
	    return self.lmList

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
    hand_detector = handDetector("hand_detector")
    rclpy.spin(hand_detector)


