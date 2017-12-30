#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Float64MultiArray, Bool, String
from sensor_msgs.msg import Image

import configparser

TOPICSTOLISTEN = []


def setup():
    config = configparser.ConfigParser()
    # config.sections()
    config.read('config.ini')
    #print config.get('DEFAULT', 'TopicsListen')
    TOPICSTOLISTEN.extend(config.get("DEFAULT", "TopicsListen").split(','))
    print "Configs Load Done"


def callback_cmd_vel(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]" % (
        msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]" % (
        msg.angular.x, msg.angular.y, msg.angular.z))
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10hz
    pub.publish(msg)

def callback_cameras(msg):
    rospy.loginfo("camera:: "+str(msg))

def listener_cmd_vel():
    print "listenning to cmd_vel"
    rospy.Subscriber("cmd_vel_fake", Twist, callback_cmd_vel)

def listener_cameras():
    print "listenning to cameras"
    rospy.Subscriber("conde_camera_tracking_right/image_raw", Image, callback_cameras)
    rospy.Subscriber("conde_camera_tracking_left/image_raw", Image, callback_cameras)

if __name__ == '__main__':
    setup()
    rospy.init_node('listener', anonymous=True)
    if "cmd_vel" in TOPICSTOLISTEN:
        listener_cmd_vel()
    if "cameras" in TOPICSTOLISTEN:
        listener_cameras()
    rospy.spin()

