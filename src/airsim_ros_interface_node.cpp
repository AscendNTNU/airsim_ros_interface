#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "common/AirSimSettings.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"

#include <memory>

typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;

msr::airlib::MultirotorRpcLibClient image_client, lidar_client;
sensor_msgs::LaserScan scan;
sensor_msgs::CameraInfo camera_info;
std::string camera_name;
int framerate = 60;
std::string frame_id = "quad";

sensor_msgs::CameraInfo generateCameraInfo(void);
sensor_msgs::LaserScan generateLaserScanMetaData(const ros::NodeHandle& node_handle);

/**
 * Retrieves the AirSim settings from the settings.json file and sets up the laser scan.
 */
void setup(const ros::NodeHandle& node_handle) {
    // --- Attaching to AirSim ---

    image_client.confirmConnection();
    lidar_client.confirmConnection();

    // Important somehow, AirSim doesn't like when you start asking for images instantly after setting up the
    // connection.
    // ros::Duration(4.0).sleep();

    image_client.enableApiControl(true);
    image_client.armDisarm(true);

    lidar_client.enableApiControl(true);
    lidar_client.armDisarm(true);

    // --- AirSim Setting loading ---

    const std::string settings_path = msr::airlib::Settings::Settings::getUserDirectoryFullPath("settings.json");
    bool found = std::ifstream(settings_path.c_str()).good();
    std::string settings_text;

    if (found) {
        std::ifstream ifs(settings_path);
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        settings_text = buffer.str();

        AirSimSettings::initializeSettings(settings_text);
        AirSimSettings::singleton().load([settings_text]() {
            Settings& settings_json = Settings::loadJSonString(settings_text);
            return settings_json.getString("SimMode", "");
        });
    } else {
        throw "Could not find the settings.json file, it should be placed in your Documents/AirSim folder.";
    }

    // --- Sensors ---
    scan = generateLaserScanMetaData(node_handle);
    camera_info = generateCameraInfo();
}

sensor_msgs::CameraInfo generateCameraInfo() {
    // Do mind that this implementation is fixed at a single quadcopter with a single camera.
    // We also only grab the first camera in the map, so this implementation is limited to a single camera with a
    // single capture setting for now.
    const auto& quadcopter = AirSimSettings::singleton().vehicles["PX4"];
    sensor_msgs::CameraInfo camera_info;
    bool is_set = false;

    for (const auto& capture_setting_element : quadcopter->cameras.begin()->second.capture_settings) {
        const auto& capture_setting = capture_setting_element.second;

        // AirSim includes some capture settings with fov of NaN for some reason
        if (std::isnan(capture_setting.fov_degrees)) {
            continue;
        }

        camera_name = quadcopter->cameras.begin()->first;

        is_set = true;

        camera_info.width = capture_setting.width;
        camera_info.height = capture_setting.height;
        camera_info.distortion_model = "plump_bob";

        // Defined as this (fx = fy = f) from the people at Unreal, see here:
        // https://github.com/unrealcv/unrealcv/issues/14#issuecomment-487346581
        float f = capture_setting.width / (2.0 * tan((M_PI / 360.0) * capture_setting.fov_degrees));

        camera_info.K = {f, 0.0, capture_setting.width / 2.0, 0.0, f, capture_setting.height / 2.0, 0.0, 0.0, 1.0};
        camera_info.P = {
            f, 0.0, capture_setting.width / 2.0, 0.0, 0.0, f, capture_setting.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
        camera_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    }

    if (!is_set) {
        throw "Could not get the fov for the camera, are the settings correct?";
    }

    // TODO: Get these from simulation, from param?

    return camera_info;
}

sensor_msgs::LaserScan generateLaserScanMetaData(const ros::NodeHandle& node_handle) {
    sensor_msgs::LaserScan scan;

    const auto& quadcopter = AirSimSettings::singleton().vehicles["PX4"];
    bool is_set = false;

    for (const auto& sensor_element : quadcopter->sensors) {
        const auto& name = sensor_element.first;
        const auto& setting = sensor_element.second;

        if (setting->sensor_type == SensorBase::SensorType::Lidar) {
            auto lidar_setting = *static_cast<msr::airlib::AirSimSettings::LidarSetting*>(setting.get());
            scan.angle_min = lidar_setting.horizontal_FOV_start * M_PI / 180.0 - M_PI;
            scan.angle_max = lidar_setting.horizontal_FOV_end * M_PI / 180.0 - M_PI;
            node_handle.param<float>("angle_increment", scan.angle_increment, M_PI / 180.0);
            scan.scan_time = 1.0 / lidar_setting.horizontal_rotation_frequency;
            scan.range_min = 0;
            scan.range_max = lidar_setting.range;
            scan.time_increment = 0.0;
            is_set = true;
        }
    }

    if (!is_set) {
        throw "Could not find the lidar, is it specified in the settings?";
    }

    return scan;
}

void publishImage(const image_transport::CameraPublisher& camera_publisher,
                  const sensor_msgs::CameraInfoPtr& camera_info_ptr, const std::string& frame) {
    // Retrieve uncompressed images, see the AirSim API documentation for more info
    const std::vector<ImageResponse>& image_responses =
        image_client.simGetImages({ImageRequest(camera_name, ImageType::Scene, false, false)});

    if (!image_responses.empty()) {
        const ImageResponse& image_response = image_responses[0];

        sensor_msgs::ImagePtr image_message_ptr = boost::make_shared<sensor_msgs::Image>();
        image_message_ptr->data = image_response.image_data_uint8;
        image_message_ptr->step = image_response.width * 3;
        image_message_ptr->header.stamp = ros::Time::now();
        image_message_ptr->header.frame_id = frame;
        image_message_ptr->height = image_response.height;
        image_message_ptr->width = image_response.width;
        image_message_ptr->encoding = "bgr8";
        image_message_ptr->is_bigendian = 0;

        camera_info_ptr->header.frame_id = frame;
        camera_info_ptr->header.stamp = ros::Time::now();

        camera_publisher.publish(image_message_ptr, camera_info_ptr);
    }
}

geometry_msgs::PoseStamped pose;

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) { pose = *msg; }

void publishLidarScan(const ros::Publisher& scan_publisher, const std::string& frame) {
    const msr::airlib::LidarData& lidar_data = lidar_client.getLidarData();

    scan.header.frame_id = frame;
    scan.header.stamp = ros::Time::now();

    const unsigned int ranges_size = std::ceil((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.assign(ranges_size, scan.range_max);

    for (int i = 0; i < lidar_data.point_cloud.size(); i += 3) {
        // The scan is in NED (local frame), so we have to convert to ENU and make it to local frame so
        // that the check for range is valid
        const double dx = lidar_data.point_cloud[i + 1] - pose.pose.position.x;
        const double dy = lidar_data.point_cloud[i] - pose.pose.position.y;
        const double dz = -lidar_data.point_cloud[i + 2] - pose.pose.position.z;

        const double range = sqrt(dx * dx + dy * dy + dz * dz);
        const double angle = atan2(dy, dx);
        const unsigned int index = (angle - scan.angle_min) / scan.angle_increment;

        if (index >= ranges_size || index < 0) {
            continue;
        }

        if (range < scan.ranges[index]) {
            scan.ranges[index] = range;
        }
    }

    scan_publisher.publish(scan);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "airsim_ros_interface_node");
    ros::NodeHandle node_handle;

    image_transport::ImageTransport it(node_handle);
    image_transport::CameraPublisher camera_publisher = it.advertiseCamera("airsim/camera/image", 1);

    ros::Subscriber pose_subscriber =
        node_handle.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &poseCallback);
    ros::Publisher scan_publisher = node_handle.advertise<sensor_msgs::LaserScan>("/scan", 1);

    try {
        try {
            setup(node_handle);
        } catch (const char* message) {
            ROS_FATAL_STREAM(message);
            return 0;
        }

        ROS_INFO_STREAM("Setup completed, starting publishing");

        node_handle.param("framerate", framerate, 60);
        node_handle.param<std::string>("frame_id", frame_id, "base_link");

        sensor_msgs::CameraInfoPtr camera_info_ptr = boost::make_shared<sensor_msgs::CameraInfo>(camera_info);


        ros::Rate rate(framerate);

        double time_counter = 0;
        ros::Time last_time = ros::Time::now();

        while (ros::ok()) {
            if ((ros::Time::now() - last_time).toSec() >= 0.1) {
                publishLidarScan(scan_publisher, frame_id);
                last_time = ros::Time::now();
            }

            publishImage(camera_publisher, camera_info_ptr, frame_id);

            ros::spinOnce();
            rate.sleep();
        }
    } catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }

    lidar_client.reset();

    image_client.enableApiControl(false);
    image_client.armDisarm(false);

    lidar_client.enableApiControl(false);
    lidar_client.armDisarm(false);

    return 0;
}
