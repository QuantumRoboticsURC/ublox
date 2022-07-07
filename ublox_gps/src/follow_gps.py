#!/usr/bin/env python
import math
import csv
import rospy
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#Camara ZED Disminuye sentido horario. NORTE = 0 grados 
#Camara ZED Aumenta sentido anti horario ESTE = -90 grados
#Norte 19.634669, -99.074926,  4.81 grados
#Sur 19.62164,  -99.073953, -177 grados
#Este 19.624768, -99.067749, -96 grados
#Oeste 19.626214, -99.077013, 75 grados  
#Se tuvo que poner menos al angulo que se calcula para que quede con el de la ZED
#Se inicializa viendo hacia el norte 

#Se inicializa variable row
row = 0 #A partir de que waypoint va a iniciar
roll = pitch = yaw = 0.0
target_angle = 0.0
zed_angle = 0.0
kp = 1.0
distance = 0.0

def callback(data):
    global row, roll, pitch, yaw, target_angle, zed_angle, distance

    file = open("gps_coordinates.csv")
    csvreader = csv.reader(file)
    header = next(csvreader)
    #print(header)
    csvreader = csv.reader(file)
    rows = list(csvreader)
    
    if (row <= len(rows)-2):
        delta_lat = float(rows[row][1])-data.pose.pose.position.x
        delta_long = float(rows[row][2])-data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        zed_angle = yaw*180/math.pi
        target_angle = -math.atan2(delta_long, delta_lat)*180/math.pi

        distance = ((delta_lat**2 + delta_long**2)**0.5)*108000
        if (distance < 2):
            row += 1
#        print ("Posicion actual        : Lat ", data.pose.pose.position.x, ", Long: ", data.pose.pose.position.y)
#        print ("Posicion objetivo      : Lat ", rows[row][1], ", Long: ", rows[row][2])
#        print ("Angulo de actual (ZED) : ", zed_angle)
#        print ("Angulo objetivo        : ", target_angle)
#        print ("Distancia              : ", distance)
#        print ()
        file.close()
    else:
        print ("Termine")
        rospy.is_shutdown()


def listener():

    rospy.init_node('gps_distance', anonymous=True)
    rospy.Subscriber("combined_odom", Odometry, callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(10)
    command = Twist()
    while not rospy.is_shutdown():


        error = target_angle-zed_angle
        if (error > -5 and error < 5):
            if (distance < 5 ):
                command.linear.x = 0.2
                command.angular.z = 0.0
                print("Avanzando recto velocidad minima, distancia restante: ", distance)
            else:
                command.linear.x = 0.4
                command.angular.z = 0.0
                print("Avanzando recto velocidad maxima, distancia restante: ", distance)
        else:
            print("Girando.... target={} current:{} error:{}", target_angle, zed_angle, error )
            vel_angular_z = kp * (target_angle-zed_angle)        
            vel_mapeada = (vel_angular_z - -180) * (1 - -1) / (180 - -180) + -1;

            if (vel_mapeada > 0.5):
                command.angular.z = 0.5
            elif (vel_mapeada < -0.5):
                command.angular.z = -0.5
            else:
                command.linear.x = 0
                command.angular.z = vel_mapeada
            pub.publish(command)
            




        r.sleep()

if __name__ == '__main__':
    listener()
