#!/usr/bin/env python3

import zmq
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Listener(object):
    def __init__(self):
        rospy.init_node("angles", anonymous=True)
        self.publisher = rospy.Publisher("/ROV/thrusters", Int32MultiArray, queue_size=10)
        
        c1 = zmq.Context()
        self.thrus = c1.socket(zmq.PAIR) 
        c2 = zmq.Context()
        self.pub = c2.socket(zmq.PAIR)
        self.pub.bind('tcp://127.0.0.1:1122')

    def callback(self,msg):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        yaw = euler_from_quaternion(quaternion)[2]
        wz = msg.angular_velocity.z

        imu = f"{wz},{yaw}".encode("utf-8")
        self.pub.send(imu)

    def thrus_callback(self,timer):
        self.thrus.connect('tcp://127.0.0.1:1234')
        phi = self.thrus.recv().decode("utf-8")
        print(phi)
        phi = phi.split(",")
        phi1 = int(phi[0])
        phi2 = int(phi[1])
        phi3 = int(phi[2])
        phi4 = int(phi[3])
        phi5 = int(phi[4])
        phi6 = int(phi[5])

        vel = Int32MultiArray()
        vel.data = [phi1, phi2, phi3, phi4, phi5, phi6]
        rospy.loginfo(vel)
        self.publisher.publish(vel)

    def listener(self):

        self.timer = rospy.Timer(rospy.Duration(0.01), self.thrus_callback)
        rospy.Subscriber("/imu/data", Imu, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    l= Listener()
    l.listener()