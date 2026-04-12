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
import gc
import threading
import enum

def distance(point_1, point_2 ):
    """
    计算两个点间欧氏距离
    :param point_1: 点1
    :param point_2: 点2
    :return: 两点间的距离
    """
    if len(point_1) != len(point_2):
        raise ValueError("两点的维度不一致")
    return math.sqrt(sum([(point_2[i] - point_1[i]) ** 2 for i in range(len(point_1))]))

def vector_2d_angle(v1, v2):
    """
    计算两向量间的夹角 -pi ~ pi
    :param v1: 第一个向量
    :param v2: 第二个向量
    :return: 角度
    """
    norm_v1_v2 = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos = v1.dot(v2) / (norm_v1_v2)
    sin = np.cross(v1, v2) / (norm_v1_v2)
    angle = np.degrees(np.arctan2(sin, cos))
    return angle


def get_area_max_contour(contours, threshold=100):
    """
    获取轮廓中面积最重大的一个, 过滤掉面积过小的情况
    :param contours: 轮廓列表
    :param threshold: 面积阈值, 小于这个面积的轮廓会被过滤
    :return: 如果最大的轮廓面积大于阈值则返回最大的轮廓, 否则返回None
    """
    contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
    contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
    if len(contour_area) > 0:
        max_c_a = max(contour_area, key=lambda c_a: c_a[1])
        return max_c_a
    return None


def get_hand_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标
    :param img: 像素坐标对应的图片
    :param landmarks: 归一化的关键点
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)


def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度
    :param landmarks: 手部关键点
    :return: 各个手指的角度
    """
    angle_list = []
    # thumb 大拇指
    angle_ = vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2])
    angle_list.append(angle_)
    # index 食指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8])
    angle_list.append(angle_)
    # middle 中指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12])
    angle_list.append(angle_)
    # ring 无名指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16])
    angle_list.append(angle_)
    # pink 小拇指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20])
    angle_list.append(angle_)
    angle_list = [abs(a) for a in angle_list]
    return angle_list


def h_gesture(angle_list):
    """
    通过二维特征确定手指所摆出的手势
    :param angle_list: 各个手指弯曲的角度
    :return : 手势名称字符串
    """
    thr_angle, thr_angle_thumb, thr_angle_s = 65.0, 53.0, 49.0
    if (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "five"
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
    else:
        gesture_str = "none"
    return gesture_str


def draw_points(img, points, tickness=4, color=(255, 0, 0)):
    """
    将记录的点连线画在画面上
    """
    points = np.array(points).astype(dtype=np.int32)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, tuple(p), tuple(points[i + 1]), color, tickness)

def get_track_img(points):
    """
    用记录的点生成一张黑底白线的轨迹图
    """
    points = np.array(points).astype(dtype=np.int32)
    x_min, y_min = np.min(points, axis=0).tolist()
    x_max, y_max = np.max(points, axis=0).tolist()
    track_img = np.full([y_max - y_min + 100, x_max - x_min + 100, 1], 0, dtype=np.uint8)
    points = points - [x_min, y_min]
    points = points + [50, 50]
    draw_points(track_img, points, 1, (255, 255, 255))
    return track_img




class State(enum.Enum):
    NULL = 0
    TRACKING = 1
    RUNNING = 2



class Hand(Node):
    def __init__(self,name):
        super().__init__(name)
		
        self.drawing = mp.solutions.drawing_utils
        self.timer = time.time()
        self.move_state = False
        self.state = State.NULL
        self.points = []
        self.start_count = 0
        self.no_finger_timestamp = time.time()
        self.gc_stamp = time.time()
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6
        )

        self.rgb_bridge = CvBridge()
        self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
        self.pub_SingleTargetAngle = self.create_publisher(ArmJoint, "arm_joint", 10)
        self.init_joints = [90, 164, 18, 0, 90, 30]
        self.pubSix_Arm(self.init_joints)
        self.sub_rgb = self.create_subscription(Image,"/camera/color/image_raw",self.get_RGBImageCallBack,100)



    def pubSingleArm(self, joint_id,joint_angle,run_time=2000):
        arm_joint =ArmJoint()
        arm_joint.joint = joint_angle
        arm_joint.id = joint_id
        arm_joint.time = run_time
        self.pub_SingleTargetAngle.publish(arm_joint)

    def arm_move_triangle(self):
        move_joints = [90, 131, 52, 0, 90, 180]
        self.pubSix_Arm(move_joints)
        time.sleep(1.5)

        move_joints = [45, 180, 0, 0, 90, 180]
        self.pubSix_Arm(move_joints)
        time.sleep(1.5)

        move_joints = [135, 180, 0, 0, 90, 180]
        self.pubSix_Arm(move_joints)
        time.sleep(1.5)


        move_joints  = [90, 131, 52, 0, 90, 180]
        self.pubSix_Arm(move_joints)
        time.sleep(1.5)


    
    def arm_move_square(self):
        move_joints = [90, 0, 180, 20, 90, 30]
        self.pubSix_Arm(move_joints)
        time.sleep(1.4)
        for i in range(3):
            self.pubSingleArm(4,-15)
            time.sleep(0.4)
            self.pubSingleArm(4,20)
            time.sleep(0.4)

    def arm_move_circle(self):
        for i in range(5):
            self.pubSingleArm(5,60)
            time.sleep(0.4)
            self.pubSingleArm(5,120)
            time.sleep(0.4)
        self.pubSingleArm(5,90)
        time.sleep(0.4)

    def arm_move_star(self):
        for i in range(3):
            move_joints = [90, 131, 52, 0, 90, 180]
            self.pubSix_Arm(move_joints)
            time.sleep(1.2)
            move_joints =  [90, 164, 18, 0, 90, 30]
            self.pubSix_Arm(move_joints)
            time.sleep(1)

	

    def arm_move_action(self, name):
        time.sleep(1)
        print("-----------------")
        if name == 'Triangle':
            self.arm_move_triangle()
        elif name == 'Square':
            self.arm_move_square()
        elif name == 'Circle':
            self.arm_move_circle()
        elif name == 'Star':
            self.arm_move_star()
        self.pubSix_Arm(self.init_joints)
        time.sleep(1.5)
        self.move_state = False



	
    def get_RGBImageCallBack(self,msg):
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, "bgr8")
        rgb_image = cv2.flip(rgb_image, 1) # 水平翻转
        result_image = np.copy(rgb_image)
        if self.timer <= time.time() and self.state == State.RUNNING:
            self.state = State.NULL
        try:
            results = self.hand_detector.process(rgb_image) if self.state != State.RUNNING else None
            if results is not None and results.multi_hand_landmarks:
                gesture = "none"
                index_finger_tip = [0, 0]
                self.no_finger_timestamp = time.time()  # 记下当期时间，以便超时处理
                for hand_landmarks in results.multi_hand_landmarks:
                    self.drawing.draw_landmarks(
                        result_image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS)
                    landmarks = get_hand_landmarks(rgb_image, hand_landmarks.landmark)
                    angle_list = (hand_angle(landmarks))
                    gesture = (h_gesture(angle_list))
                    index_finger_tip = landmarks[8].tolist()

                if self.state == State.NULL:
                    if gesture == "one":  # 检测到单独伸出食指，其他手指握拳
                        self.start_count += 1
                        if self.start_count > 20:
                            self.state = State.TRACKING
                            self.points = []
                    else:
                        self.start_count = 0

                elif self.state == State.TRACKING:
                    if gesture == "five": # 伸开五指结束画图
                        self.state = State.NULL
                        
                        # 生成黑白轨迹图
                        track_img = get_track_img(self.points)

                        contours = cv2.findContours(track_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

                        contour = get_area_max_contour(contours, 300)
					
                        contour = contour[0]
                        # 按轨迹图识别所画图形
                        # cv2.fillPoly在图像上绘制并填充多边形
                        track_img = cv2.fillPoly(track_img, [contour,], (255, 255, 255))

                        for _ in range(3):
                            # 腐蚀函数
                            track_img = cv2.erode(track_img, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

                            # 膨胀函数
                            track_img = cv2.dilate(track_img, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

                        contours = cv2.findContours(track_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

                        contour = get_area_max_contour(contours, 300)

                        contour = contour[0]
                        h, w = track_img.shape[:2]

                        track_img = np.full([h, w, 3], 0, dtype=np.uint8)
                        track_img = cv2.drawContours(track_img, [contour, ], -1, (0, 255, 0), 2)

                        # 对图像轮廓点进行多边形拟合
                        approx = cv2.approxPolyDP(contour, 0.026 * cv2.arcLength(contour, True), True)

                        track_img = cv2.drawContours(track_img, [approx, ], -1, (0, 0, 255), 2)

                        graph_name = 'unknown'
                        
                        print(len(approx))
                        # 根据轮廓包络的顶点数确定图形
                        if len(approx) == 3:
                            graph_name = 'Triangle'
                        if len(approx) == 4 or len(approx) == 5:
                            graph_name = 'Square'
                        if 5 < len(approx) < 10:
                            graph_name = 'Circle'
                        if len(approx) == 10:
                            graph_name = 'Star'
                        cv2.putText(track_img, graph_name, (10, 40),cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 2)
                        cv2.imshow('track', track_img)
                        if not self.move_state:
                            self.move_state = True
                            task = threading.Thread(target=self.arm_move_action, name="arm_move_action", args=(graph_name, ))
                            task.setDaemon(True)
                            task.start()

                    else:
                        if len(self.points) > 0:
                            if distance(self.points[-1], index_finger_tip) > 5:
                                self.points.append(index_finger_tip)
                        else:
                            self.points.append(index_finger_tip)

                    draw_points(result_image, self.points)
                else:
                    pass
            else:
                if self.state == State.TRACKING:
                    if time.time() - self.no_finger_timestamp > 2:
                        self.state = State.NULL
                        self.points = []

        except BaseException as e:
            print(e)
			
        #result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        key = cv2.waitKey(1) 

        if key == ord(' '): # 按空格清空已经记录的轨迹
            self.points = []
        if time.time() > self.gc_stamp:
            self.gc_stamp = time.time() + 1
            gc.collect()		
		

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
    hand_ = Hand("hand_detector")
    rclpy.spin(hand_)

