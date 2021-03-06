#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <csignal>
#include <memory>

#include "common/AirSimSettings.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef msr::airlib::MultirotorRpcLibClient AirSimClient;

geometry_msgs::PoseStamped pose;

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) { pose = *msg; }

void publishImage(const ros::Publisher& image_publisher, const ros::Publisher& depth_image_publisher,
                  AirSimClient& image_client, const std::string& frame, const std::string& camera_name) {
    // Retrieve uncompressed images, see the AirSim API documentation for more info
    const std::vector<ImageResponse>& image_responses =
        image_client.simGetImages({ImageRequest(camera_name, ImageType::Scene, false, false),
                                   ImageRequest(camera_name + "_depth", ImageType::DepthVis, true, false)});
    sensor_msgs::Image image, depth_image;

    if (image_responses.size() == 2) {
        const ImageResponse& image_response = image_responses[0];
        const ImageResponse& depth_image_response = image_responses[1];

        image.header.frame_id = frame;
        image.encoding = "bgr8";
        image.is_bigendian = 0;
        image.header.stamp = ros::Time::now();

        image.data = image_response.image_data_uint8;
        image.step = image_response.width * 3;
        image.height = image_response.height;
        image.width = image_response.width;

        // TODO: this is really bad
        std::vector<unsigned char> bytes;
        for (int i = 0; i < depth_image_response.image_data_float.size(); i++) {
            bytes.push_back(static_cast<unsigned char>(depth_image_response.image_data_float[i] * 255));
        }

        depth_image.header.frame_id = frame;
        depth_image.encoding = "mono8";
        depth_image.is_bigendian = 0;
        depth_image.header.stamp = ros::Time::now();
        depth_image.data = bytes;
        depth_image.step = depth_image_response.width;
        depth_image.height = depth_image_response.height;
        depth_image.width = depth_image_response.width;
    }

    image_publisher.publish(image);
    depth_image_publisher.publish(depth_image);
}

/**
 * @brief Grabs the data from the given lidar given by @p laser_name from @p vehicle_name and fills the @p
 *        out_laser_scan.
 *
 * @param out_laser_scan Where the data is filled (passed by reference, so it will overwrite)
 * @param lidar_client The AirSim client to grab the data from.
 * @param laser_name The name of the laser/lidar.
 * @param vehicle_name The name of the vehicle hosting the laser/lidar.
 */
void publishLaserScan(const ros::Publisher& publisher, AirSimClient& lidar_client, const std::string& frame,
                      const std::string& lidar_name, const std::string& vehicle_name) {
    const msr::airlib::LidarData& lidar_data = lidar_client.getLidarData(lidar_name, vehicle_name);

    sensor_msgs::LaserScan laser_scan;
    laser_scan.header.frame_id = frame;

    laser_scan.angle_max = M_PI;
    laser_scan.angle_min = -M_PI;
    laser_scan.angle_increment = M_PI / 180.0;
    laser_scan.scan_time = 0.1;
    laser_scan.range_max = 50;
    laser_scan.time_increment = 0;
    laser_scan.header.stamp = ros::Time::now();

    const unsigned int ranges_size =
        std::ceil((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment);
    laser_scan.ranges.assign(ranges_size, laser_scan.range_max);

    for (unsigned int i = 0; i < lidar_data.point_cloud.size(); i += 3) {
        // The scan is in NED (local frame), so we have to convert to ENU and make it to local frame so
        // that the check for range is valid
        const double dx = lidar_data.point_cloud[i + 1] - pose.pose.position.x;
        const double dy = lidar_data.point_cloud[i] - pose.pose.position.y;
        const double dz = -lidar_data.point_cloud[i + 2] - pose.pose.position.z;

        const double range = sqrt(dx * dx + dy * dy + dz * dz);
        const double angle = atan2(dy, dx);
        const unsigned int index = (angle - laser_scan.angle_min) / laser_scan.angle_increment;

        if (index >= ranges_size || index < 0) {
            continue;
        }

        if (range < laser_scan.ranges[index]) {
            laser_scan.ranges[index] = range;
        }
    }

    publisher.publish(laser_scan);
}

void publishPointCloud(const ros::Publisher& publisher, AirSimClient& point_cloud_client, const std ::string& frame,
                       const std::string& lidar_name, const std::string& vehicle_name) {
    auto const point_cloud_data = point_cloud_client.getLidarData(lidar_name, vehicle_name);
    sensor_msgs::PointCloud2 point_cloud;
    point_cloud.header.frame_id = frame;

    if (point_cloud_data.point_cloud.size() > 3) {
        point_cloud.height = 1;
        point_cloud.width = point_cloud_data.point_cloud.size() / 3;

        point_cloud.fields.resize(3);
        point_cloud.fields[0].name = "x";
        point_cloud.fields[1].name = "y";
        point_cloud.fields[2].name = "z";
        int offset = 0;

        for (size_t d = 0; d < point_cloud.fields.size(); ++d, offset += 4) {
            point_cloud.fields[d].offset = offset;
            point_cloud.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
            point_cloud.fields[d].count = 1;
        }

        point_cloud.is_bigendian = false;
        point_cloud.point_step = offset;
        point_cloud.row_step = point_cloud.point_step * point_cloud.width;

        point_cloud.is_dense = true;
        std::vector<float> data_std = point_cloud_data.point_cloud;

        for (unsigned int i = 0; i < data_std.size(); i += 3) {
            // Convert from NED to ENU
            const float x = data_std[i + 1] - pose.pose.position.x;
            const float y = data_std[i] - pose.pose.position.y;
            const float z = -(data_std[i + 2] - 0.8) - pose.pose.position.z;

            data_std[i + 0] = x;
            data_std[i + 1] = y;
            data_std[i + 2] = z;
        }

        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&data_std[0]);
        vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
        point_cloud.data = std::move(lidar_msg_data);
    }

    publisher.publish(point_cloud);
}

AirSimClient image_client, lidar_client, point_cloud_client;

void signalHandler(int signum) {
    image_client.reset();
    exit(signum);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "airsim_ros_interface_node");
    ros::NodeHandle node_handle;

    signal(SIGINT, signalHandler);

    ros::Subscriber pose_subscriber =
        node_handle.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &poseCallback);

    ros::Publisher scan_publisher = node_handle.advertise<sensor_msgs::LaserScan>("/scan", 1);
    ros::Publisher point_cloud_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("/airsim/point_cloud", 1);
    ros::Publisher image_publisher = node_handle.advertise<sensor_msgs::Image>("airsim/camera/image", 1);
    ros::Publisher depth_image_publisher = node_handle.advertise<sensor_msgs::Image>("airsim/camera/depth", 1);

    try {
        image_client.confirmConnection();
        lidar_client.confirmConnection();
        point_cloud_client.confirmConnection();

        std::string frame, vehicle_name, camera_name, lidar_2d_name, lidar_3d_name;
        int refresh_rate;
        int scan_and_point_cloud_publishing_rate;

        const std::string prefix = ros::this_node::getName();

        if (!node_handle.getParam(prefix + "/rate", refresh_rate)) {
            ROS_FATAL_STREAM("Could not get parameter rate");
            ros::shutdown();
            return 1;
        }
        if (!node_handle.getParam(prefix + "/scan_and_point_cloud_publishing_rate",
                                  scan_and_point_cloud_publishing_rate)) {
            ROS_FATAL_STREAM("Could not get parameter scan_and_point_cloud_publishing_rate");
            ros::shutdown();
            return 1;
        }

        if (!node_handle.getParam(prefix + "/frame", frame)) {
            ROS_FATAL_STREAM("Could not get parameter frame");
            ros::shutdown();
            return 1;
        }

        if (!node_handle.getParam(prefix + "/vehicle_name", vehicle_name)) {
            ROS_FATAL_STREAM("Could not get parameter vehicle_name");
            ros::shutdown();
            return 1;
        }

        if (!node_handle.getParam(prefix + "/camera_name", camera_name)) {
            ROS_FATAL_STREAM("Could not get parameter camera_name");
            ros::shutdown();
            return 1;
        }

        if (!node_handle.getParam(prefix + "/lidar_2d_name", lidar_2d_name)) {
            ROS_FATAL_STREAM("Could not get parameter lidar_2d_name");
            ros::shutdown();
            return 1;
        }

        if (!node_handle.getParam(prefix + "/lidar_3d_name", lidar_3d_name)) {
            ROS_FATAL_STREAM("Could not get parameter lidar_3d_name");
            ros::shutdown();
            return 1;
        }

        ROS_INFO_STREAM("Setup completed, starting publishing");

        ros::Rate rate(refresh_rate);

        double time_counter = 0;
        ros::Time last_time = ros::Time::now();

        while (ros::ok()) {
            if ((ros::Time::now() - last_time).toSec() >=
                1.0 / static_cast<float>(scan_and_point_cloud_publishing_rate)) {
                publishLaserScan(scan_publisher, lidar_client, frame, lidar_2d_name, vehicle_name);
                publishPointCloud(point_cloud_publisher, point_cloud_client, frame, lidar_3d_name, vehicle_name);
                last_time = ros::Time::now();
            }

            publishImage(image_publisher, depth_image_publisher, image_client, frame, camera_name);

            ros::spinOnce();
            rate.sleep();
        }
    } catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }

    // One of the clients has to reset AirSim
    image_client.reset();

    return 0;
}
