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
import random
import numpy
import argparse
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

FAULTTYPE = ""

CAMERAQUEUE = Queue.Queue()

CONFIG = configparser.ConfigParser()

bridge = CvBridge()

FREEZEIMG = None

FREEZECOUNTER = 0


def publisher_camera(item_right, item_left, true_topic=False):
    if true_topic:
        pub_right = rospy.Publisher(
            "conde_camera_tracking_right/image_raw", Image, queue_size=10)
        pub_left = rospy.Publisher(
            "conde_camera_tracking_left/image_raw", Image, queue_size=10)
    else:
        pub_right = rospy.Publisher(
            "conde_camera_tracking_right/image_raw_fake", Image, queue_size=10)
        pub_left = rospy.Publisher(
            "conde_camera_tracking_left/image_raw_fake", Image, queue_size=10)
    #camera_queue_pub(item_right, "right")
    #camera_queue_pub(item_left, "left")
    origin_time = rospy.Time.now()
    item_left.header.stamp = origin_time
    item_right.header.stamp = origin_time
    rospy.Rate(10)  # 10hz

    pub_right.publish(item_right)
    pub_left.publish(item_left)


def worker():
    while True:
        item = CAMERAQUEUE.get()
        item_right = item[1]
        item_left = item[0]
        if FAULTTYPE == "SLOW":
            time.sleep(CONFIG.getint("SLOW", "time") / 1000.0)
        elif FAULTTYPE == "RANDOM":
            np_arr_right = numpy.fromstring(item_right.data, numpy.uint8)
            np_arr_right.tolist()
            random.shuffle(np_arr_right)
            rRandom = numpy.array(np_arr_right)
            item_right.data = rRandom.tostring()

            np_arr_left = numpy.fromstring(item_left.data, numpy.uint8)
            np_arr_left.tolist()
            random.shuffle(np_arr_left)
            lRandom = numpy.array(np_arr_left)
            item_left.data = lRandom.tostring()
            publisher_camera(item_right, item_left)
        elif FAULTTYPE == "FREEZE":
            if FREEZECOUNTER == 0:
                FREEZEIMG = item
            if FREEZECOUNTER < CONFIG.getint("FREEZE", "iterations"):
                FREEZECOUNTER += 1
            else:
                publisher_camera(FREEZEIMG[1], FREEZEIMG[0])
                FREEZECOUNTER = 0
        elif FAULTTYPE == "REPLACE":
            rospy.loginfo("NotImplemented")
        else:
            rospy.loginfo("NotImplemented")
            exit()

        CAMERAQUEUE.task_done()


def inject_payload():
    imgpath_right = CONFIG.get("INJECTPAYLOAD", "path_right")
    imgpath_left = CONFIG.get("INJECTPAYLOAD", "path_left")

    stream_right = open(imgpath_right, "rb")
    bytes = bytearray(stream_right.read())
    numpyarray_right = numpy.asarray(bytes, dtype=numpy.uint8)
    image_np_right = cv2.imdecode(
        numpyarray_right, cv2.CV_LOAD_IMAGE_GRAYSCALE)

    stream_left = open(imgpath_left, "rb")
    bytes = bytearray(stream_left.read())
    numpyarray_left = numpy.asarray(bytes, dtype=numpy.uint8)
    image_np_left = cv2.imdecode(numpyarray_left, cv2.CV_LOAD_IMAGE_GRAYSCALE)

    item_right = Image()
    item_right.header.stamp = rospy.Time.now()
    item_right.header.frame_id = "camera_link_tracking"
    #item_right.format = "jpeg"
    item_right.data = numpy.array(cv2.imencode(
        '.jpg', image_np_right)[1]).tostring()

    item_left = item_right
    item_left.data = numpy.array(cv2.imencode(
        '.jpg', image_np_left)[1]).tostring()

    while(True):
        publisher_camera(item_right, item_left, True)
        time.sleep(CONFIG.getint("INJECTPAYLOAD", "interval") / 1000.0)


def camera_store_callback_right(msg):
    rospy.loginfo("Received an image.")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        cv2.imwrite('images/camera_image_right_' +
                    str(rospy.Time.now()) + '.jpeg', cv2_img)


def camera_store_callback_left(msg):
    rospy.loginfo("Received an image.")
    print msg.header
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        cv2.imwrite('images/camera_image_left_' +
                    str(rospy.Time.now()) + '.jpeg', cv2_img)


def camera_store():
    image_right = rospy.Subscriber(
        "conde_camera_tracking_right/image_raw", Image, camera_store_callback_right)
    image_left = rospy.Subscriber(
        "conde_camera_tracking_left/image_raw", Image, camera_store_callback_left)


def setup():
    # config.sections()
    CONFIG.read('config.ini')
    rospy.loginfo("Configuration loading done.")


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
    #rospy.loginfo("left:: " + str(msg_left) + "\nright:: " + str(msg_left))


def listener_cmd_vel():
    print "listenning to cmd_vel"
    rospy.Subscriber("cmd_vel_fake", Twist, callback_cmd_vel)


def listener_cameras():
    print "listenning to cameras"

    image_right = message_filters.Subscriber(
        "conde_camera_tracking_right/image_raw", Image)
    image_left = message_filters.Subscriber(
        "conde_camera_tracking_left/image_raw", Image)

    ts = message_filters.TimeSynchronizer([image_left, image_right], 40)
    ts.registerCallback(callback_cameras)


def init(fault_type):
    rospy.init_node('listener', anonymous=True)
    rospy.loginfo("Starting...")
    FAULTTYPE = fault_type
    setup()
    if FAULTTYPE == "INJECTPAYLOAD":
        inject_payload()
    elif FAULTTYPE == "CAMERASTORE":
        camera_store()
    else:
        listener_cmd_vel()
        listener_cameras()

        t = threading.Thread(target=worker)
        t.daemon = True
        t.start()
    rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='ROS queue fault-injection toolkit')
    parser.add_argument(
        '-f', '--faulttype', help='Fault Type: RANDOM, SLOW, FREEZE', required=True, default="None")

    args = parser.parse_args()
    init(args.faulttype)
