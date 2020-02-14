#!/usr/bin/env python

##################################
# This file is quite a dirty hack
# (we blame Unreal for being hard
#  to interface with)
##################################

import os
import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseWithCovariance,PoseWithCovarianceStamped, Pose, Point

def translate_module_position(text):
    # Input text is of the form "X=0.0 Y=0.0 Z=0.0 T=0.0"
    axes = text.split(" ")
    x = float(axes[0][2:])
    y = float(axes[1][2:])
    z = float(axes[2][2:])
    t = float(axes[3][2:])
    return x, y, z, t

def strToPoseWithCovariance(str):
    x,y,z,t = translate_module_position(str)
    msg = PoseWithCovarianceStamped()
    msg.header = Header()
    msg.header.stamp.secs = int(t)
    msg.header.stamp.nsecs = int((t%1)*1000000000.0)
    msg.pose = PoseWithCovariance()
    msg.pose.pose = Pose()
    msg.pose.pose.position = Point(x,y,z)
    return msg

def main():
    rospy.init_node("module_position_publisher")
    module_position_publisher = rospy.Publisher("/airsim/module_position", PoseWithCovarianceStamped, queue_size=1)

    path = rospy.get_param("~data_file_path", "/mnt/c/tmp/RosIntegrationUnreal/RosIntegrationData.txt")
    rate = rospy.Rate(rospy.get_param("~rate", 10));

    while not rospy.is_shutdown():

        try:
            file = open(path, "r")

            while True: 
                data = file.read()
                if data != "":
                    module_position_publisher.publish(strToPoseWithCovariance(data))
                    break

            file.close()
            os.remove(path)

        except OSError as error:
            print("Could not open file for reading: " + error + ". Is the simulation running?")

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

