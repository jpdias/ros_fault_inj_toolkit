#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Float64MultiArray, Bool, String
from sensor_msgs.msg import Image
import Queue
import configparser
import threading
import time
import message_filters

TOPICSTOLISTEN = []

CAMERAQUEUE = Queue.Queue()

CONFIG = configparser.ConfigParser()


def camera_queue_pub(msg, topic):
    if topic == "right":
        pub = rospy.Publisher(
            "conde_camera_tracking_right/image_raw_fake", Image, queue_size=10)
    elif topic == "left":
        pub = rospy.Publisher(
            "conde_camera_tracking_left/image_raw_fake", Image, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    pub.publish(msg)


def worker():
    while True:
        item = CAMERAQUEUE.get()
        item_right = item[1]
        item_left = item[0]
        if CONFIG.getint("DEFAULT", "Time"):
            time.sleep(CONFIG.getint("DEFAULT", "Time"))
        camera_queue_pub(item_right, "right")
        camera_queue_pub(item_left, "left")
        CAMERAQUEUE.task_done()


T = threading.Thread(target=worker)
T.daemon = True
T.start()


def setup():
    # config.sections()
    CONFIG.read('config.ini')
    # print config.get('DEFAULT', 'TopicsListen')
    TOPICSTOLISTEN.extend(CONFIG.get("DEFAULT", "TopicsListen").split(','))
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


def callback_cameras(msg_left, msg_right):
    t = msg_left, msg_right
    CAMERAQUEUE.put(t)
    rospy.loginfo("left:: " + str(msg_left) + "\nright:: " + str(msg_left))


def listener_cmd_vel():
    print "listenning to cmd_vel"
    rospy.Subscriber("cmd_vel_fake", Twist, callback_cmd_vel)


def listener_cameras():
    print "listenning to cameras"

    image_right =  rospy.Subscriber("conde_camera_tracking_right/image_raw",
                     Image)
    image_left = rospy.Subscriber("conde_camera_tracking_left/image_raw",
                     Image)

    ts = message_filters.TimeSynchronizer([image_left, image_right], 40)
    ts.registerCallback(callback_cameras)

if __name__ == '__main__':
    setup()
    rospy.init_node('listener', anonymous=True)
    if "cmd_vel" in TOPICSTOLISTEN:
        listener_cmd_vel()
    if "cameras" in TOPICSTOLISTEN:
        listener_cameras()
    rospy.spin()
