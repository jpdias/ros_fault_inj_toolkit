#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz  
    pub.publish(msg)  


def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("cmd_vel_fake", Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()