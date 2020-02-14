#!/usr/bin/env python

import setup_path 
import airsim
import rospy
import time
import math
import signal
import sys
import msgpackrpc 

from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import Image, LaserScan, PointCloud

current_pose = PoseStamped()
rpc_client = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", 41451), timeout = 3600, pack_encoding = 'utf-8', unpack_encoding = 'utf-8')


def poseCallback(msg):
    global current_pose
    current_pose = msg

def publishLaserScanFromData(data):
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

    scan_publisher.publish(scan)

def publishPointCloudFromData(data):
    point_cloud = PointCloud()
    # TODO: Which frame?
    point_cloud.header.frame_id = "map"

    for i in range(0, len(data.point_cloud), 3):
        # The lidar data is in NED so we have to convert to ENU and transform to local frame
        # TODO: Which reference frame
        x = data.point_cloud[i + 1] - current_pose.pose.position.x 
        y = data.point_cloud[i] - current_pose.pose.position.y
        z = -data.point_cloud[i + 2] - current_pose.pose.position.z

        point = Point32()
        point.x = x
        point.y = y
        point.z = z

        point_cloud.points.append(point)

    point_cloud_publisher.publish(point_cloud)


def sendBackPropellerThrustSignal(value):
    rpc_client.call('setBackPropellerControlSignal', value, "PX4")

def collidesWithModule():
    return rpc_client.call("collidesWithModule", "SimpleFlight")

def signal_handler(sig, frame):
    # Wait until AirSim closes the connnection
    rospy.loginfo("Quitting, stopping connection to AirSim, please wait...")
    image_client.reset()
    rospy.signal_shutdown("Successfully shut down the airsim node")

def main():
    global image_client, scan_publisher, point_cloud_publisher

    rospy.init_node('airsim_ros_interface', disable_signals=True)
    signal.signal(signal.SIGINT, signal_handler)

    image_publisher = rospy.Publisher("/airsim/camera/image", Image, queue_size=1)
    scan_publisher = rospy.Publisher("/scan", LaserScan, queue_size=1)
    point_cloud_publisher = rospy.Publisher("/airsim/point_cloud", PointCloud, queue_size=1)

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseCallback)

    # We have to set up multiple clients as the AirSim API seems to prefer this, through tests this causes UE to 
    # freeze far less. No clue why this is the case.
    image_client = airsim.MultirotorClient()
    lidar_client = airsim.MultirotorClient()
    point_cloud_client = airsim.MultirotorClient()

    connected = False
    while not connected:  
        try:
            image_client.confirmConnection()
            lidar_client.confirmConnection()
            point_cloud_client.confirmConnection()

            connected = True
        except:
            rospy.logerr("Could not connect to AirSim, is the simulation running?")
            time.sleep(1.0)

    rospy.loginfo("Starting publishing!")

    rate = rospy.Rate(rospy.get_param("~rate"))

    last_scan_time = rospy.get_time()
    scan_time = 1.0 / rospy.get_param("~scan_publishing_frequency")

    last_image_time = rospy.get_time()
    image_time = 1.0 / rospy.get_param("~image_publishing_frequency") 

    last_point_cloud_time = rospy.get_time()
    point_cloud_time = 1.0 / rospy.get_param("~point_cloud_publishing_frequency") 


    frame_id = rospy.get_param("~frame_id", "base_link")

    camera_name = rospy.get_param("~camera_name")
    image_width = rospy.get_param("~image_width")
    image_height = rospy.get_param("~image_height")

    while not rospy.is_shutdown():
        
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
            publishLaserScanFromData(lidar_client.getLidarData(lidar_name = "Lidar2D", vehicle_name = "PX4"))
            last_scan_time = rospy.get_time()

        if rospy.get_time() - last_point_cloud_time >= point_cloud_time:
            publishPointCloudFromData(point_cloud_client.getLidarData(lidar_name = "Lidar3D", vehicle_name = "PX4"))
            last_point_cloud_time = rospy.get_time()

        rate.sleep()

    rospy.loginfo("Quitting, stopping connection to AirSim, please wait...")
    
    # Safely wrap up the connection to AirSim in order to prevent freezes in UE
    image_client.reset()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
