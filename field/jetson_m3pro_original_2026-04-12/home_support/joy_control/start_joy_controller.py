#开机启动文件

import subprocess
from PyQt5.QtWidgets import QApplication, QMessageBox
from PyQt5.QtCore import Qt
import psutil
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

launch_process = None
run_process = None

def kill_process_tree(pid):#关闭进程树
    try:
        parent = psutil.Process(pid)
        for child in parent.children(recursive=True):
            child.kill()
        parent.kill()
    except psutil.NoSuchProcess:
        pass
        
def close_event(event):
    event.accept()  # 接受关闭事件
    kill_process_tree(launch_process.pid)
    kill_process_tree(run_process.pid)    
    app.quit()  # 退出应用程序

def start_processes():
    global launch_process, run_process
    msg_box = QMessageBox()
    msg_box.setText("joy controller is running...")
    msg_box.setWindowTitle("joy mode ")  
    msg_box.setIcon(QMessageBox.Information)
    msg_box.setStandardButtons(QMessageBox.NoButton) 
    msg_box.setWindowModality(Qt.NonModal)  

    launch_process = subprocess.Popen(['ros2', 'launch', 'yahboomcar_ctrl', 'yahboomcar_joy_launch.py'])
    run_process = subprocess.Popen(['ros2', 'run', 'yahboomcar_ctrl', 'yahboom_joy_M3Pro'])
    print("joy controller start successful!!!")
    
    msg_box.closeEvent = close_event  # 绑定自定义关闭事件
    msg_box.show()  

    app.exec_()  

class autostart(Node):
    def __init__(self):
        super().__init__('autostart_node') 
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        twist = Twist()       
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0

        # 等待订阅者连接
        while not self.cmd_vel_pub.get_subscription_count():
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('Waiting for subscribers...')
            time.sleep(0.1) 
        self.get_logger().info('Published initial velocity command')
        self.cmd_vel_pub.publish(twist) 
if __name__ == "__main__":

    rclpy.init()
    
    auto_start = autostart()
    
    app = QApplication([])  # 初始化QApplication实例
    
    start_processes()
    
    rclpy.shutdown()