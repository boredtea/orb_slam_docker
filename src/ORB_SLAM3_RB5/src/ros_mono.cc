/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "System.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <opencv2/core/core.hpp>


using namespace std;
using std::placeholders::_1;

static int cameraWidth, cameraHeight;

class ImageGrabber: public rclcpp::Node
{
public:
    int count = 0;
    
    ImageGrabber(ORB_SLAM3::System *pSLAM) : Node("Mono") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_main/image_raw", 10, std::bind(&ImageGrabber::GrabImage, this, _1));

        std::cout << "Subscribed to /camera_main/image_raw" << std::endl;

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/camera_pose", 10);
        map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_points", 10);

        tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        mpSLAM = pSLAM;

        // Initialize defaults
        sensor_type_ = ORB_SLAM3::System::MONOCULAR;  // <-- or pass this dynamically
        world_frame_id_ = "map";
        Tc0w_ = Sophus::SE3f();
    };

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void PublishPose(const Sophus::SE3f& T, const rclcpp::Time& stamp, const string& frame_id, const string& child_frame_id);

private:
    void publish_ros_tracked_mappoints(const std::vector<ORB_SLAM3::MapPoint*>& map_points, const rclcpp::Time& msg_time);
    sensor_msgs::msg::PointCloud2 tracked_mappoints_to_pointcloud(const std::vector<ORB_SLAM3::MapPoint*>& map_points, const rclcpp::Time& msg_time);
    tf2::Transform SE3f_to_tfTransform(const Sophus::SE3f& T_SE3f);

    // Members
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    ORB_SLAM3::System *mpSLAM;

    // Added fields
    int sensor_type_;
    std::string world_frame_id_;
    Sophus::SE3f Tc0w_;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    if (argc != 3)
    {
        cerr << endl
             << "Usage: ros2 run ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;
        rclcpp::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    std::cout << "Started SLAM system setup" << std::endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);
    std::cout << "Finished SLAM system setup" << std::endl;

    // Load camera parameters from settings file
    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    cameraWidth = fSettings["Camera.width"].operator int();
    cameraHeight = fSettings["Camera.height"].operator int();

    auto node = std::make_shared<ImageGrabber>(&SLAM);

    rclcpp::spin(node);

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();

    return 0;
}


void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "A message is received");

    // Copy the ROS image message to cv::Mat.
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat img;
    if (cv_ptr->image.cols != cameraWidth or cv_ptr->image.rows != cameraHeight) {
        RCLCPP_INFO(this->get_logger(), "Resize image from (%d, %d) to (%d, %d).", cv_ptr->image.cols, cv_ptr->image.rows, cameraWidth, cameraHeight);
        cv::resize(cv_ptr->image, img, cv::Size(cameraWidth, cameraHeight), cv::INTER_LINEAR);
    } else {
        img = cv_ptr->image;
    }

    // Track image using ORB-SLAM3
    double time_in_seconds = double(cv_ptr->header.stamp.sec) + double(cv_ptr->header.stamp.nanosec) * 1e-9;
    Sophus::SE3f se3_tf = mpSLAM->TrackMonocular(img, time_in_seconds);

    // Publish pose
    string frame_id("map");
    string child_frame_id("camera_");
    child_frame_id = child_frame_id + std::to_string(count);
    count += 1;
    rclcpp::Time stamp = msg->header.stamp;
    PublishPose(se3_tf, stamp, frame_id, child_frame_id);

    // Publish map points (tracked keypoints)
    std::vector<ORB_SLAM3::MapPoint*> map_points = mpSLAM->GetTrackedMapPoints();
    publish_ros_tracked_mappoints(map_points, stamp);
}



// code reference
// https://github.com/uzh-rpg/rpg_vikit/blob/10871da6d84c8324212053c40f468c6ab4862ee0/vikit_ros/src/output_helper.cpp#L15 
// void ImageGrabber::PublishPose(
//     const Sophus::SE3f& T, 
//     const rclcpp::Time& stamp,
//     const string& frame_id, 
//     const string& child_frame_id,
//     tf2_ros::TransformBroadcaster& tf_br)
// {
//     tf2::Transform transform_msg;
//     Eigen::Quaternionf q(T.rotationMatrix());
//     transform_msg.setOrigin(tf2::Vector3(T.translation().x(), T.translation().y(), T.translation().z()));
//     tf2::Quaternion tf_q; tf_q.setX(q.x()); tf_q.setY(q.y()); tf_q.setZ(q.z()); tf_q.setW(q.w());
//     transform_msg.setRotation(tf_q);
//     tf_br.sendTransform(tf2::StampedTransform(transform_msg, stamp, frame_id, child_frame_id));
// }

// void ImageGrabber::PublishPose(
//     const Sophus::SE3f& T, 
//     const rclcpp::Time& stamp,
//     const string& frame_id, 
//     const string& child_frame_id)
// {
//     Eigen::Quaternionf q(T.rotationMatrix());
//     geometry_msgs::msg::TransformStamped tf_gm;
//     geometry_msgs::msg::PoseStamped msg;
    
//     tf_gm.header.stamp = stamp;
//     tf_gm.header.frame_id = frame_id;
//     tf_gm.child_frame_id = child_frame_id;

//     tf_gm.transform.translation.x = T.translation().x();
//     tf_gm.transform.translation.y = T.translation().y();
//     tf_gm.transform.translation.z = T.translation().z();

//     tf_gm.transform.rotation.x = q.x();
//     tf_gm.transform.rotation.y = q.y();
//     tf_gm.transform.rotation.z = q.z();
//     tf_gm.transform.rotation.w = q.w();

//     // populate msg
//     msg.header.stamp = stamp;
//     msg.header.frame_id = frame_id;
//     msg.pose.position.x = T.translation().x();
//     msg.pose.position.y = T.translation().y();
//     msg.pose.position.z = T.translation().z();

//     msg.pose.orientation.x = q.x();
//     msg.pose.orientation.y = q.y();
//     msg.pose.orientation.z = q.z();
//     msg.pose.orientation.w = q.w();
//     // output_array.push_back(msg);
    
//     std::cout << "HALOOOOOO before publishin \n" << std::endl;
//     pose_pub_->publish(msg);
//     std::cout << "after publishin \n" << std::endl;

//     // send TF
//     tf_->sendTransform(tf_gm);
//     RCLCPP_INFO(this->get_logger(), "Sending TF");
//     //RCLCPP_INFO("Transformation published for marker.");
    
// }

void ImageGrabber::PublishPose(
    const Sophus::SE3f& T, 
    const rclcpp::Time& stamp,
    const string& frame_id, 
    const string& child_frame_id)
{
    // Rotate the pose before publishing it (e.g., 90 deg around X)
    Eigen::Matrix3f R_correction;
    R_correction = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX());
    Sophus::SE3f T_corrected(R_correction * T.rotationMatrix(), T.translation());

    Eigen::Quaternionf q(T_corrected.rotationMatrix());
    geometry_msgs::msg::TransformStamped tf_gm;
    geometry_msgs::msg::PoseStamped msg;
    
    tf_gm.header.stamp = stamp;
    tf_gm.header.frame_id = frame_id;
    tf_gm.child_frame_id = child_frame_id;

    tf_gm.transform.translation.x = T_corrected.translation().x();
    tf_gm.transform.translation.y = T_corrected.translation().y();
    tf_gm.transform.translation.z = T_corrected.translation().z();

    tf_gm.transform.rotation.x = q.x();
    tf_gm.transform.rotation.y = q.y();
    tf_gm.transform.rotation.z = q.z();
    tf_gm.transform.rotation.w = q.w();

    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.pose.position.x = T_corrected.translation().x();
    msg.pose.position.y = T_corrected.translation().y();
    msg.pose.position.z = T_corrected.translation().z();

    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();

    pose_pub_->publish(msg);
    tf_->sendTransform(tf_gm);
}


void ImageGrabber::publish_ros_tracked_mappoints(const std::vector<ORB_SLAM3::MapPoint*>& map_points, const rclcpp::Time& msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = tracked_mappoints_to_pointcloud(map_points, msg_time);
    map_points_pub_->publish(cloud);
}

sensor_msgs::msg::PointCloud2 ImageGrabber::tracked_mappoints_to_pointcloud(const std::vector<ORB_SLAM3::MapPoint*>& map_points, const rclcpp::Time& msg_time)
{
    const int num_channels = 3; // x y z

    if (map_points.empty())
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = msg_time;
    cloud.header.frame_id = world_frame_id_;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = cloud.data.data();

    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {
            Eigen::Vector3f pMPw = map_points[i]->GetWorldPos();

            if (sensor_type_ == ORB_SLAM3::System::MONOCULAR || sensor_type_ == ORB_SLAM3::System::STEREO)
            {
                Sophus::SE3f Tc0mp(Eigen::Matrix3f::Identity(), pMPw);
                Sophus::SE3f Twmp = Tc0w_.inverse() * Tc0mp;
                pMPw = Twmp.translation();
            }

            tf2::Vector3 point_translation(pMPw.x(), pMPw.y(), pMPw.z());

            float data_array[num_channels] = {
                point_translation.x(),
                point_translation.y(),
                point_translation.z()
            };

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
        }
    }

    return cloud;
}

// OPTIONAL — not yet used but added for future
tf2::Transform ImageGrabber::SE3f_to_tfTransform(const Sophus::SE3f& T_SE3f)
{
    Eigen::Matrix3f R_mat = T_SE3f.rotationMatrix();
    Eigen::Vector3f t_vec = T_SE3f.translation();

    tf2::Matrix3x3 R_tf(
        R_mat(0, 0), R_mat(0, 1), R_mat(0, 2),
        R_mat(1, 0), R_mat(1, 1), R_mat(1, 2),
        R_mat(2, 0), R_mat(2, 1), R_mat(2, 2)
    );
    tf2::Vector3 t_tf(t_vec.x(), t_vec.y(), t_vec.z());

    tf2::Transform tf_transform;
    tf_transform.setBasis(R_tf);
    tf_transform.setOrigin(t_tf);

    return tf_transform;
}
