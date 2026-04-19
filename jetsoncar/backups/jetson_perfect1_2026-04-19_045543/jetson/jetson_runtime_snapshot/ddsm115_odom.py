#!/usr/bin/env python3
import math, struct, threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
def crc8(data):
    crc=0x00
    for b in data:
        crc^=b
        for _ in range(8):
            if crc&0x01:crc=(crc>>1)^0x8C
            else:crc>>=1
    return crc
def yq(yaw):
    q=Quaternion();q.z=math.sin(yaw/2);q.w=math.cos(yaw/2);return q
class Odom(Node):
    L=[1,3];R=[2,4]
    def __init__(self):
        super().__init__("ddsm115_odom")
        self.wr=0.050;self.tw=0.400
        self.x=0.0;self.y=0.0;self.th=0.0
        self.lt=self.get_clock().now()
        self.tq=0;self.fq=0
        self.ser=None;self.lk=threading.Lock()
        try:
            self.ser=serial.Serial("/dev/ttyUSB0",115200,timeout=0.03)
            self.get_logger().info("RS485 opened")
        except Exception as e:
            self.get_logger().error(f"Serial fail: {e}")
            self.get_logger().warn("Zero odom mode")
        self.pub=self.create_publisher(Odometry,"/odom",10)
        self.create_timer(0.1,self.tick)
        self.create_timer(10.0,self.stats)
    def qry(self,mid):
        f=bytes([mid,0x74,0,0,0,0,0,0,0]);return f+bytes([crc8(f)])
    def ask(self,mid):
        if not self.ser or not self.ser.is_open:return None
        self.tq+=1
        with self.lk:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(self.qry(mid))
                r=self.ser.read(10)
                if len(r)==10 and r[0]==mid and crc8(r[:9])==r[9]:
                    return float(struct.unpack_from(">h",r,4)[0])
                self.fq+=1
            except:self.fq+=1
        return None
    def sv(self,ids,neg):
        rpms=[r for mid in ids if(r:=self.ask(mid))is not None]
        if not rpms:return 0.0
        avg=sum(rpms)/len(rpms)
        if neg:avg=-avg
        return avg*2*math.pi*self.wr/60.0
    def tick(self):
        now=self.get_clock().now()
        dt=(now-self.lt).nanoseconds/1e9;self.lt=now
        if dt<=0 or dt>0.5:return
        vl=self.sv(self.L,False);vr=self.sv(self.R,True)
        v=(vl+vr)/2;w=(vr-vl)/self.tw
        if abs(w)<1e-6:
            self.x+=v*math.cos(self.th)*dt;self.y+=v*math.sin(self.th)*dt
        else:
            r=v/w;self.x+=r*(math.sin(self.th+w*dt)-math.sin(self.th))
            self.y-=r*(math.cos(self.th+w*dt)-math.cos(self.th))
        self.th+=w*dt;self.th=math.atan2(math.sin(self.th),math.cos(self.th))
        m=Odometry();m.header.stamp=now.to_msg()
        m.header.frame_id="odom";m.child_frame_id="base_link"
        m.pose.pose.position.x=self.x;m.pose.pose.position.y=self.y
        m.pose.pose.orientation=yq(self.th)
        m.twist.twist.linear.x=v;m.twist.twist.angular.z=w
        pc=[0.0]*36;pc[0]=0.1;pc[7]=0.1;pc[14]=1e6;pc[21]=1e6;pc[28]=1e6;pc[35]=0.3
        m.pose.covariance=pc
        tc=[0.0]*36;tc[0]=0.05;tc[7]=1e6;tc[35]=0.15
        m.twist.covariance=tc;self.pub.publish(m)
    def stats(self):
        if self.tq>0:self.get_logger().info(f"q={self.tq} fail={100*self.fq/self.tq:.1f}% pos=({self.x:.2f},{self.y:.2f}) th={math.degrees(self.th):.1f}")
        elif self.ser is None:self.get_logger().warn("No serial")
def main():
    rclpy.init();n=Odom()
    try:rclpy.spin(n)
    except:pass
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=="__main__":main()
