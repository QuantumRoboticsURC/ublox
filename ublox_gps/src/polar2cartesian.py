#! /usr/bin/env python

import rospy
import numpy as np


def callback (data):
	pass

latitude = 0	
longitude = 0

rospy.init_node("polar2cartesian")

if __name__ == "__main__":
    rho = input("Rho: ")
    phi = input("Phi: ")
    
    latitude = rho * np.cos(phi)
    longitude = rho * np.sin(phi)

    print ("lat: " + latitude +"; long: " + longitude + ";")


