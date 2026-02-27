#pragma once
#ifndef SUPER_ODOMETRY_VISUAL_ODOMETRY_H
#define SUPER_ODOMETRY_VISUAL_ODOMETRY_H

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include "super_odometry/config/parameter.h"

namespace super_odometry {

class visualOdometry : public rclcpp::Node {
public:
    explicit visualOdometry(const rclcpp::NodeOptions &options);

    void initInterface();

private:
    bool readParameters();
    void cameraInfoHandler(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void imageHandler(const sensor_msgs::msg::Image::SharedPtr msg);

    void resetTracking(const cv::Mat &gray, const rclcpp::Time &stamp);
    void detectFeatures(const cv::Mat &gray, std::vector<cv::Point2f> &points) const;
    void publishOdometry(const rclcpp::Time &stamp);
    void publishDebugOutputs(const rclcpp::Time &stamp, const cv::Mat &gray, int detected_features,
                             int tracked_features, int inlier_features, bool tracking_ok,
                             const std::vector<cv::Point2f> &overlay_points);

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_visual_odom_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_detected_features_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_tracked_features_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_inlier_features_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_tracking_ok_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_image_;

    std::string image_topic_;
    std::string camera_info_topic_;
    std::string odom_topic_;
    std::string frame_id_;
    std::string child_frame_id_;

    int max_features_;
    int min_tracked_features_;
    int min_inliers_;
    int max_lost_frames_;
    double ransac_threshold_px_;
    double translation_scale_;
    bool publish_debug_topics_;
    bool publish_debug_image_;
    std::string debug_image_topic_;

    bool camera_ready_;
    bool initialized_;
    int lost_frames_;

    cv::Mat K_;
    cv::Mat prev_gray_;
    std::vector<cv::Point2f> prev_points_;
    rclcpp::Time prev_stamp_;

    Eigen::Quaterniond q_w_curr_;
    Eigen::Vector3d t_w_curr_;
    cv::Ptr<cv::ORB> orb_detector_;
};

} // namespace super_odometry

#endif // SUPER_ODOMETRY_VISUAL_ODOMETRY_H
