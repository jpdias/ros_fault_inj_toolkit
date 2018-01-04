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


class Counter:
    def __init__(self):
        self.lock = threading.Lock()
        self.value = 0

    def increment(self):
        self.lock.acquire()
        self.value = value = self.value + 1
        self.lock.release()
        return value

    def reset(self):
        self.value = 0
        self.value = value = self.value
        return value

    def get(self):
        self.value = value = self.value
        return value


FREEZECOUNTER = Counter()

FAULTTYPE = []

CAMERAQUEUE = Queue.Queue()

CONFIG = configparser.ConfigParser()

bridge = CvBridge()

FREEZEIMG = None


def publisher_camera(item_right, item_left, true_topic=False):
    if true_topic:
        pub_right = rospy.Publisher(
            CONFIG.get("DEFAULTS", "camera_right_topic"), Image, queue_size=10)
        pub_left = rospy.Publisher(
            CONFIG.get("DEFAULTS", "camera_left_topic"), Image, queue_size=10)
    else:
        pub_right = rospy.Publisher(
            CONFIG.get("DEFAULTS", "camera_right_topic_fake"), Image, queue_size=10)
        pub_left = rospy.Publisher(
            CONFIG.get("DEFAULTS", "camera_left_topic_fake"), Image, queue_size=10)
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
        if FAULTTYPE[0] == "SLOW":
            time.sleep(CONFIG.getint("SLOW", "time") / 1000.0)
            publisher_camera(item_right, item_left)
        elif FAULTTYPE[0] == "RANDOM":
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
        elif FAULTTYPE[0] == "FREEZE":
            print FREEZECOUNTER.get()
            if FREEZECOUNTER.get() == 0:
                global FREEZEIMG
                FREEZEIMG = item
                FREEZECOUNTER.increment()
            elif FREEZECOUNTER.increment() < CONFIG.getint("FREEZE", "iterations"):
                pass
            else:
                FREEZECOUNTER.reset()
            publisher_camera(FREEZEIMG[1], FREEZEIMG[0])
        elif FAULTTYPE[0] == "PARTIALLOSS":
            np_arr_right = numpy.fromstring(item_right.data, numpy.uint8)
            np_arr_right.tolist()
            
            np_arr_left = numpy.fromstring(item_left.data, numpy.uint8)
            np_arr_left.tolist()
            if CONFIG.get("PARTIALLOSS", "orientation") == "horizontal":
                if CONFIG.get("PARTIALLOSS", "split") == "top":
                    for index, item in enumerate(np_arr_right):
                        if index < len(np_arr_right)/2:
                                np_arr_right[index] = 255
                    rRandom = numpy.array(np_arr_right)
                    item_right.data = rRandom.tostring()

                    for index, item in enumerate(np_arr_left):
                        if index < len(np_arr_right)/2:
                                np_arr_left[index] = 255
                    lRandom = numpy.array(np_arr_left)
                else:
                    for index, item in enumerate(np_arr_right):
                        if index > len(np_arr_right)/2:
                                np_arr_right[index] = 255
                    rRandom = numpy.array(np_arr_right)
                    item_right.data = rRandom.tostring()

                    for index, item in enumerate(np_arr_left):
                        if index > len(np_arr_right)/2:
                                np_arr_left[index] = 255
                    lRandom = numpy.array(np_arr_left)
                item_left.data = lRandom.tostring()
            elif CONFIG.get("PARTIALLOSS", "orientation") == "vertical":
                rRandom = numpy.array(np_arr_right)
                lRandom = numpy.array(np_arr_left)
                count = 0
                if CONFIG.get("PARTIALLOSS", "split") == "left":
                    for index, val in numpy.ndenumerate(rRandom):
                        if count <= (item_right.width)*3/2+2:
                            rRandom[index]=255
                            count += 1
                        elif count <= (item_right.width-0.5)*3:
                            count += 1
                        else:
                            count = 0
                if CONFIG.get("PARTIALLOSS", "split") == "right":
                    for index, val in numpy.ndenumerate(rRandom):
                        if count <= (item_right.width)*3/2+2:
                            count += 1
                        elif count <= (item_right.width-0.5)*3:
                            rRandom[index]=255
                            count += 1
                        else:
                            count = 0
                item_right.data = rRandom.tostring()

                if CONFIG.get("PARTIALLOSS", "split") == "left":
                    for index, val in numpy.ndenumerate(rRandom):
                        if count <= (item_right.width)*3/2+2:
                            lRandom[index]=255
                            count += 1
                        elif count <= (item_left.width-0.5)*3:
                            count += 1
                        else:
                            count = 0
                if CONFIG.get("PARTIALLOSS", "split") == "right":
                    for index, val in numpy.ndenumerate(rRandom):
                        if count <= (item_left.width)*3/2+2:
                            count += 1
                        elif count <= (item_left.width-0.5)*3:
                            lRandom[index]=255
                            count += 1
                        else:
                            count = 0
                item_left.data = lRandom.tostring()

                #for index, item in enumerate(np_arr_left):
                #    if index % size == 0:
                #            np_arr_left[index] = 255
                #lRandom = numpy.array(np_arr_left)
                #item_left.data = lRandom.tostring()
            elif CONFIG.get("PARTIALLOSS", "orientation") == "odd-even":
                for index, item in enumerate(np_arr_right):
                    if index < len(np_arr_right):
                        if index%2 == 0:
                            np_arr_right[index] = 255
                rRandom = numpy.array(np_arr_right)
                item_right.data = rRandom.tostring()

                for index, item in enumerate(np_arr_left):
                    if index < len(np_arr_left):
                        if index%2 == 0:
                            np_arr_left[index] = 255
                lRandom = numpy.array(np_arr_left)
                item_left.data = lRandom.tostring()

            publisher_camera(item_right, item_left)
        else:
            rospy.loginfo("NotImplemented")
            exit()

        CAMERAQUEUE.task_done()


def inject_payload():
    imgpath_right = CONFIG.get("INJECTPAYLOAD", "path_right")
    imgpath_left = CONFIG.get("INJECTPAYLOAD", "path_left")

    tt = rospy.Time.now()

    item_right = bridge.cv2_to_imgmsg(cv2.imread(imgpath_right), 'bgr8')
    item_right.header.stamp = tt
    item_right.header.frame_id = "camera_link_tracking"
    
    item_left = bridge.cv2_to_imgmsg(cv2.imread(imgpath_left), 'bgr8')
    item_left.header.stamp = tt
    item_left.header.frame_id = "camera_link_tracking"

    #print item_right.data

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
        cv2.imwrite(CONFIG.get("CAMERASTORE", "path") + '/camera_image_right_' +
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
        cv2.imwrite(CONFIG.get("CAMERASTORE", "path") + '/camera_image_left_' +
                    str(rospy.Time.now()) + '.jpeg', cv2_img)


def camera_store():
    image_right = rospy.Subscriber(
        CONFIG.get("DEFAULTS", "camera_right_topic"), Image, camera_store_callback_right)
    image_left = rospy.Subscriber(
        CONFIG.get("DEFAULTS", "camera_left_topic"), Image, camera_store_callback_left)


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
    rospy.Subscriber(CONFIG.get("DEFAULTS", "cmd_vel"), Twist, callback_cmd_vel)


def listener_cameras():
    print "listenning to cameras"

    image_right = message_filters.Subscriber(
        CONFIG.get("DEFAULTS", "camera_right_topic"), Image)
    image_left = message_filters.Subscriber(
        CONFIG.get("DEFAULTS", "camera_left_topic"), Image)

    ts = message_filters.TimeSynchronizer([image_left, image_right], 40)
    ts.registerCallback(callback_cameras)


def init(fault_type):
    rospy.init_node('listener', anonymous=True)
    rospy.loginfo("Starting...")
    FAULTTYPE.append(str(fault_type))
    setup()
    if FAULTTYPE[0] == "INJECTPAYLOAD":
        inject_payload()
    elif FAULTTYPE[0] == "CAMERASTORE":
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
        description='ROS queue fault-injection toolkit', formatter_class=argparse.RawTextHelpFormatter)
    help_info = '''Fault Type: RANDOM, SLOW, FREEZE, INJECTPAYLOAD, CAMERASTORE, PARTIALLOSS

                RANDOM: Shuffles the camera feed images bytes
                SLOW: Introduces a slowness in the camara image stream
                FREEZE: Freezes a camara feed for n interations
                INJECTPAYLOAD: Injects personalized images into the queue
                CAMERASTORE: Stores all images in camera feeds
                PARTIALLOSS: Loss of part of the image

                -- Additional configuration in config.ini file.
                '''
    parser.add_argument(
        '-f', '--faulttype', help=help_info, required=True, default="None")

    args = parser.parse_args()
    init(args.faulttype)
