#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


def callback_zed(zed):
    global odom_msg
    odom_msg.pose.pose.orientation.x = zed.pose.pose.orientation.x               #// identity quaternion
    odom_msg.pose.pose.orientation.y = zed.pose.pose.orientation.y               #// identity quaternion
    odom_msg.pose.pose.orientation.z = zed.pose.pose.orientation.z               #// identity quaternion
    odom_msg.pose.pose.orientation.w = zed.pose.pose.orientation.w

def callback_gps(gps):
    global odom_msg
    odom_msg.pose.pose.position.x = gps.latitude
    odom_msg.pose.pose.position.y = gps.longitude
    odom_msg.pose.pose.position.z = gps.altitude

rospy.init_node('combined_odom', anonymous = True)

odom_pub = rospy.Publisher('combined_odom', Odometry, queue_size = 1)
odom = rospy.Subscriber('zed2/zed_node/odom', Odometry, callback_zed)
gps = rospy.Subscriber('ublox/fix', NavSatFix, callback_gps)
odom_msg = Odometry()
r = rospy.Rate(10)

while not rospy.is_shutdown():
    odom_pub.publish(odom_msg)
    r.sleep()


