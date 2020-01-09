#!/usr/bin/env python

import setup_path 
import airsim
import rospy
import time
import math
import signal
import sys

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu

current_pose = PoseStamped()

def poseCallback(msg):
    global current_pose
    current_pose = msg

def generateLaserScan(data):
    scan = LaserScan()
    scan.header.frame_id = "base_link"
    scan.angle_max =  math.pi 
    scan.angle_min = -math.pi
    scan.angle_increment = math.pi / 180.0
    scan.scan_time = 0.1
    scan.range_max = 50
    scan.time_increment = 0
    scan.header.stamp = rospy.Time.now()

    scan.ranges = [scan.range_max] * (int) ((scan.angle_max - scan.angle_min) / scan.angle_increment)

    if len(data.point_cloud) <= 2:
        return scan

    for i in range(0, len(data.point_cloud), 3):
        # The lidar data is in NED so we have to convert to ENU and transform to local frame
        dx = data.point_cloud[i + 1] - current_pose.pose.position.x 
        dy = data.point_cloud[i] - current_pose.pose.position.y
        dz = -data.point_cloud[i + 2] - current_pose.pose.position.z

        rng = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        angle = math.atan2(dy, dx)
        index = int((angle - scan.angle_min) / scan.angle_increment)

        if index < 0 or index >= len(scan.ranges):
            continue

        if rng < scan.ranges[index]:
            scan.ranges[index] = rng

    return scan

def signal_handler(sig, frame):
    # Wait until AirSim closes the connnection
    rospy.loginfo("Quitting, stopping connection to AirSim, please wait...")
    imu_client.reset()
    rospy.signal_shutdown("Successfully shut down the airsim node")

def main():
    global imu_client
    rospy.init_node('airsim_ros_interface', disable_signals=True)
    signal.signal(signal.SIGINT, signal_handler)

    imu_publisher = rospy.Publisher("/airsim/imu/data", Imu, queue_size=1)
    image_publisher = rospy.Publisher("/airsim/camera/image", Image, queue_size=1)
    scan_publisher = rospy.Publisher("/scan", LaserScan, queue_size=1)

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseCallback)

    # We have to set up multiple clients as the AirSim API seems to prefer this, through tests this causes UE to 
    # freeze far less
    imu_client = airsim.MultirotorClient()
    image_client = airsim.MultirotorClient()
    lidar_client = airsim.MultirotorClient()

    connected = False

    while not connected:  
        try:
            imu_client.confirmConnection()
            image_client.confirmConnection()
            lidar_client.confirmConnection()
            connected = True
        except:
            rospy.logerr("Could not connect to AirSim, is the simulation running?")
            time.sleep(1.0)

    rospy.loginfo("Starting publishing!")

    rate = rospy.Rate(rospy.get_param("~framerate", 100.0))

    last_imu_time = rospy.get_time()
    imu_time = 1.0 / rospy.get_param("~imu_publishing_frequency", 100.0);

    last_scan_time = rospy.get_time()
    scan_time = 1.0 / rospy.get_param("~scan_publishing_frequency", 10.0)

    last_image_time = rospy.get_time()
    image_time = 1.0 / rospy.get_param("~image_publishing_frequency", 30.0) 

    frame_id = rospy.get_param("~frame_id", "base_link")

    camera_name = rospy.get_param("~camera_name", "MyCamera1")
    image_width = rospy.get_param("~image_width", 256)
    image_height = rospy.get_param("~image_height", 144)

    while not rospy.is_shutdown():

        if rospy.get_time() - last_imu_time >= imu_time:

            imu_data = imu_client.getImuData()

            imu_msg = Imu()
            imu_msg.header.frame_id = frame_id
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.orientation.x = imu_data.orientation.x_val
            imu_msg.orientation.y = imu_data.orientation.y_val
            imu_msg.orientation.z = imu_data.orientation.z_val
            imu_msg.orientation.w = imu_data.orientation.w_val
            imu_msg.angular_velocity.x = imu_data.angular_velocity.x_val
            imu_msg.angular_velocity.y = imu_data.angular_velocity.y_val
            imu_msg.angular_velocity.z = imu_data.angular_velocity.z_val
            imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x_val
            imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y_val
            imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z_val

            imu_publisher.publish(imu_msg)

        if rospy.get_time() - last_image_time >= image_time:
            responses = image_client.simGetImages([airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)])

            for response in responses:
                img_rgb_string = response.image_data_uint8

            msg = Image() 
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame_id
            msg.encoding = "bgr8"
            msg.height = image_height 
            msg.width = image_width
            msg.data = img_rgb_string
            msg.is_bigendian = 0
            msg.step = msg.width * 3

            image_publisher.publish(msg)
            last_image_time = rospy.get_time()

        if rospy.get_time() - last_scan_time >= scan_time:
            scan = generateLaserScan(lidar_client.getLidarData())
            scan_publisher.publish(scan)
            last_scan_time = rospy.get_time()

        rate.sleep()

    rospy.loginfo("Quitting, stopping connection to AirSim, please wait...")

    # Safely wrap up the connection to AirSim in order to prevent freezes in UE
    imu_client.reset()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
