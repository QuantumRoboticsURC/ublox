#!/usr/bin/env python
import math
import csv
import rospy
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import NavSatFix

#Camara ZED Disminuye sentido horario. NORTE = 0 grados 
#Camara ZED Aumenta sentido anti horario ESTE = -90 grados
#Norte 19.634669, -99.074926,  4.81 grados
#Sur 19.62164,  -99.073953, -177 grados
#Este 19.624768, -99.067749, -96 grados
#Oeste 19.626214, -99.077013, 75 grados  
#Se tuvo que poner menos al angulo que se calcula para que quede con el de la ZED
#Se inicializa viendo hacia el norte 

#Se inicializa variable row
row = 0
roll = pitch = yaw = 0.0
target_angle = 0.0
zed_angle = 0.0
kp = 1.0
distance = 0.0
flag = 0

def callback(data):
    global row, roll, pitch, yaw, target_angle, zed_angle, distance, goal_position, flag

    file = open("/home/quantum/catkin_ws/src/ublox/ublox_gps/src/csv_files/cedetec1.csv")
    csvreader = csv.reader(file)
    header = next(csvreader)
    #print(header)
    csvreader = csv.reader(file)
    rows = list(csvreader)
  
    if (row <= len(rows)-2):
	goal_position.latitude = float(rows[row][1])
	goal_position.longitude = float(rows[row][2])
        goal_position_publisher.publish(goal_position)

        delta_lat = float(rows[row][1])-data.pose.pose.position.x
        delta_long = float(rows[row][2])-data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
#	print("yaw: " + str(yaw*180/math.pi)) 
        zed_angle = yaw*180/math.pi
        target_angle = -math.atan2(delta_long, delta_lat)*180/math.pi

        internal_error_angle = target_angle - zed_angle
        #print(internal_error_angle)

        if (internal_error_angle > -5 and internal_error_angle < 5):
            print ("Goal: ", row, " achieved... waiting 10 seconds for next goal")
            row += 1

        print ("Posicion actual        : Lat ", data.pose.pose.position.x, ", Long: ", data.pose.pose.position.y)
        print ("Posicion objetivo      : Lat ", rows[row][1], ", Long: ", rows[row][2])
        print ("Angulo de actual (IMU) : ", zed_angle)
        print ("Angulo objetivo        : ", target_angle)
        print ()
        file.close()
    else:
        print ("Termine")
	flag = 1




rospy.init_node('gps_distance', anonymous=True)
rospy.Subscriber("combined_odom", Odometry, callback)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
goal_position_publisher = rospy.Publisher('ublox/gps_goal', NavSatFix, queue_size = 1)
r = rospy.Rate(10)
command = Twist()
goal_position = NavSatFix()

while not rospy.is_shutdown():
    error = target_angle-zed_angle
    #print("Girando.... target: "+ str(target_angle)+" current: "+ str(zed_angle) + " error: " + str(error))
    print(target_angle-zed_angle)
    vel_angular_z = -kp * (target_angle-zed_angle)        
    vel_mapeada = (vel_angular_z - -180) * (1 - -1) / (180 - -180) + -1;

    speed = 0.05
    if (vel_mapeada > speed):
        command.angular.z = speed
    elif (vel_mapeada < -speed):
        command.angular.z = -speed
    else:
        command.linear.x = 0
        command.angular.z = vel_mapeada
 #   pub.publish(command)

    if flag:
        command.angular.z = 0
  #      pub.publish(command)
        rospy.is_shutdown()

    r.sleep()

