#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

latitud = 0
longitud = 0
altitud = 0
def callback_gps(data):
    global latitud
    latitud = data.latitude
    longitud = data.longitude
    altitud = data.altitude
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.latitude)

def callback_odom(data):
    global latitud
    print (latitud)
    

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('odom_publisher', anonymous=True)
    rospy.Subscriber("zed2/zed_node/odom", Odometry, callback_odom)
    rospy.Subscriber("ublox/fix", NavSatFix, callback_gps)

    # spin() simply keeps python from exiting until this node is stopped

    rospy.spin()

if __name__ == '__main__':
    listener()

