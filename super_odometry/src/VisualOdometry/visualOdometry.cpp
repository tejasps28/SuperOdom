#include "super_odometry/VisualOdometry/visualOdometry.h"

#include <algorithm>
#include <cctype>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

namespace super_odometry {

visualOdometry::visualOdometry(const rclcpp::NodeOptions &options)
    : Node("visual_odometry_node", options),
      max_features_(1200),
      min_tracked_features_(120),
      min_inliers_(60),
      max_lost_frames_(5),
      ransac_threshold_px_(1.5),
      translation_scale_(0.0),
      publish_debug_topics_(true),
      publish_debug_image_(false),
      debug_image_topic_("/SuperOdom/vo_debug_image"),
      camera_ready_(false),
      initialized_(false),
      lost_frames_(0),
      q_w_curr_(Eigen::Quaterniond::Identity()),
      t_w_curr_(Eigen::Vector3d::Zero()) {}

void visualOdometry::initInterface() {
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_;

    if (!readGlobalparam(shared_from_this())) {
        RCLCPP_WARN(this->get_logger(),
                    "[visual_odometry_node] Failed to read global parameters. Using local defaults.");
    }

    if (!readParameters()) {
        RCLCPP_ERROR(this->get_logger(),
                     "[visual_odometry_node] Failed to read node parameters. Exiting...");
        rclcpp::shutdown();
        return;
    }

    orb_detector_ = cv::ORB::create(std::max(200, max_features_));

    rclcpp::QoS image_qos(10);
    image_qos.best_effort();
    image_qos.keep_last(10);

    sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, 10,
        std::bind(&visualOdometry::cameraInfoHandler, this, std::placeholders::_1), sub_options);

    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, image_qos,
        std::bind(&visualOdometry::imageHandler, this, std::placeholders::_1), sub_options);

    pub_visual_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    if (publish_debug_topics_) {
        pub_detected_features_ =
            this->create_publisher<std_msgs::msg::Int32>(ProjectName + "/vo_detected_features", 10);
        pub_tracked_features_ =
            this->create_publisher<std_msgs::msg::Int32>(ProjectName + "/vo_tracked_features", 10);
        pub_inlier_features_ =
            this->create_publisher<std_msgs::msg::Int32>(ProjectName + "/vo_inlier_features", 10);
        pub_tracking_ok_ = this->create_publisher<std_msgs::msg::Bool>(ProjectName + "/vo_tracking_ok", 10);
        if (publish_debug_image_) {
            pub_debug_image_ = this->create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, 10);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Visual odometry image topic: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Visual odometry camera info topic: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Visual odometry output topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Visual odometry debug topics enabled: %d", publish_debug_topics_);
    if (publish_debug_topics_ && publish_debug_image_) {
        RCLCPP_INFO(this->get_logger(), "Visual odometry debug image topic: %s", debug_image_topic_.c_str());
    }
}

bool visualOdometry::readParameters() {
    this->declare_parameter<std::string>("visual_odometry_node.image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("visual_odometry_node.camera_info_topic", "/camera/camera_info");
    this->declare_parameter<std::string>(
        "visual_odometry_node.odom_topic", ODOM_TOPIC.empty() ? std::string("/visual_odometry") : ODOM_TOPIC);
    this->declare_parameter<std::string>(
        "visual_odometry_node.frame_id", WORLD_FRAME.empty() ? std::string("map") : WORLD_FRAME);
    this->declare_parameter<std::string>(
        "visual_odometry_node.child_frame_id", SENSOR_FRAME.empty() ? std::string("sensor") : SENSOR_FRAME);
    this->declare_parameter<int>("visual_odometry_node.max_features", 1200);
    this->declare_parameter<int>("visual_odometry_node.min_tracked_features", 120);
    this->declare_parameter<int>("visual_odometry_node.min_inliers", 60);
    this->declare_parameter<int>("visual_odometry_node.max_lost_frames", 5);
    this->declare_parameter<double>("visual_odometry_node.ransac_threshold_px", 1.5);
    this->declare_parameter<double>("visual_odometry_node.translation_scale", 0.0);
    this->declare_parameter<bool>("visual_odometry_node.publish_debug_topics", true);
    this->declare_parameter<bool>("visual_odometry_node.publish_debug_image", false);
    this->declare_parameter<std::string>(
        "visual_odometry_node.debug_image_topic",
        (ProjectName.empty() ? std::string("/SuperOdom") : ProjectName) + std::string("/vo_debug_image"));

    image_topic_ = this->get_parameter("visual_odometry_node.image_topic").as_string();
    camera_info_topic_ = this->get_parameter("visual_odometry_node.camera_info_topic").as_string();
    odom_topic_ = this->get_parameter("visual_odometry_node.odom_topic").as_string();
    frame_id_ = this->get_parameter("visual_odometry_node.frame_id").as_string();
    child_frame_id_ = this->get_parameter("visual_odometry_node.child_frame_id").as_string();
    max_features_ = this->get_parameter("visual_odometry_node.max_features").as_int();
    min_tracked_features_ = this->get_parameter("visual_odometry_node.min_tracked_features").as_int();
    min_inliers_ = this->get_parameter("visual_odometry_node.min_inliers").as_int();
    max_lost_frames_ = this->get_parameter("visual_odometry_node.max_lost_frames").as_int();
    ransac_threshold_px_ = this->get_parameter("visual_odometry_node.ransac_threshold_px").as_double();
    translation_scale_ = this->get_parameter("visual_odometry_node.translation_scale").as_double();
    publish_debug_topics_ = this->get_parameter("visual_odometry_node.publish_debug_topics").as_bool();
    publish_debug_image_ = this->get_parameter("visual_odometry_node.publish_debug_image").as_bool();
    debug_image_topic_ = this->get_parameter("visual_odometry_node.debug_image_topic").as_string();

    max_features_ = std::max(max_features_, 200);
    min_tracked_features_ = std::max(min_tracked_features_, 40);
    min_inliers_ = std::max(min_inliers_, 20);
    max_lost_frames_ = std::max(max_lost_frames_, 1);

    return true;
}

void visualOdometry::cameraInfoHandler(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (msg->k[0] <= 1e-9) {
        return;
    }

    K_ = (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2], msg->k[3], msg->k[4], msg->k[5], msg->k[6],
          msg->k[7], msg->k[8]);
    camera_ready_ = true;
}

void visualOdometry::detectFeatures(const cv::Mat &gray, std::vector<cv::Point2f> &points) const {
    points.clear();
    if (gray.empty()) {
        return;
    }

    std::vector<cv::KeyPoint> keypoints;
    orb_detector_->detect(gray, keypoints);
    if (keypoints.empty()) {
        return;
    }

    if (static_cast<int>(keypoints.size()) > max_features_) {
        keypoints.resize(max_features_);
    }
    cv::KeyPoint::convert(keypoints, points);
}

void visualOdometry::resetTracking(const cv::Mat &gray, const rclcpp::Time &stamp) {
    prev_gray_ = gray.clone();
    detectFeatures(prev_gray_, prev_points_);
    prev_stamp_ = stamp;
    initialized_ = true;
    lost_frames_ = 0;
}

void visualOdometry::publishOdometry(const rclcpp::Time &stamp) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = frame_id_;
    odom_msg.child_frame_id = child_frame_id_;

    odom_msg.pose.pose.position.x = t_w_curr_.x();
    odom_msg.pose.pose.position.y = t_w_curr_.y();
    odom_msg.pose.pose.position.z = t_w_curr_.z();
    odom_msg.pose.pose.orientation.x = q_w_curr_.x();
    odom_msg.pose.pose.orientation.y = q_w_curr_.y();
    odom_msg.pose.pose.orientation.z = q_w_curr_.z();
    odom_msg.pose.pose.orientation.w = q_w_curr_.w();

    for (int i = 0; i < 36; ++i) {
        odom_msg.pose.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0] = translation_scale_ > 0.0 ? 0.1 : 5.0;
    odom_msg.pose.covariance[7] = translation_scale_ > 0.0 ? 0.1 : 5.0;
    odom_msg.pose.covariance[14] = translation_scale_ > 0.0 ? 0.2 : 5.0;
    odom_msg.pose.covariance[21] = 0.05;
    odom_msg.pose.covariance[28] = 0.05;
    odom_msg.pose.covariance[35] = 0.05;

    pub_visual_odom_->publish(odom_msg);
}

void visualOdometry::publishDebugOutputs(const rclcpp::Time &stamp, const cv::Mat &gray, int detected_features,
                                         int tracked_features, int inlier_features, bool tracking_ok,
                                         const std::vector<cv::Point2f> &overlay_points) {
    if (!publish_debug_topics_) {
        return;
    }

    std_msgs::msg::Int32 detected_msg;
    detected_msg.data = detected_features;
    pub_detected_features_->publish(detected_msg);

    std_msgs::msg::Int32 tracked_msg;
    tracked_msg.data = tracked_features;
    pub_tracked_features_->publish(tracked_msg);

    std_msgs::msg::Int32 inlier_msg;
    inlier_msg.data = inlier_features;
    pub_inlier_features_->publish(inlier_msg);

    std_msgs::msg::Bool status_msg;
    status_msg.data = tracking_ok;
    pub_tracking_ok_->publish(status_msg);

    if (!publish_debug_image_ || !pub_debug_image_ || gray.empty()) {
        return;
    }

    cv::Mat debug_bgr;
    cv::cvtColor(gray, debug_bgr, cv::COLOR_GRAY2BGR);
    for (const auto &point : overlay_points) {
        cv::circle(debug_bgr, point, 2, tracking_ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 180, 255), -1);
    }

    auto debug_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, debug_bgr).toImageMsg();
    debug_msg->header.stamp = stamp;
    debug_msg->header.frame_id = frame_id_;
    pub_debug_image_->publish(*debug_msg);
}

void visualOdometry::imageHandler(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!camera_ready_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "visual_odometry_node waiting for camera_info on %s",
                             camera_info_topic_.c_str());
        return;
    }

    cv::Mat gray;
    try {
        if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
            const auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
            gray = cv_ptr->image;
        } else {
            const auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            gray = cv_ptr->image;
        }
    } catch (const cv_bridge::Exception &e) {
        // Fallback for malformed image metadata (seen in some bags):
        // infer channels from step and convert manually.
        if (msg->height == 0 || msg->width == 0 || msg->step == 0 || msg->data.empty()) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "visual_odometry_node cv_bridge conversion failed: %s", e.what());
            return;
        }

        const int inferred_channels = std::max(1, static_cast<int>(msg->step / msg->width));
        if (inferred_channels == 1 &&
            msg->data.size() >= static_cast<size_t>(msg->height) * static_cast<size_t>(msg->step)) {
            cv::Mat raw(msg->height, msg->width, CV_8UC1,
                        const_cast<unsigned char *>(msg->data.data()), msg->step);
            gray = raw.clone();
        } else if (inferred_channels >= 3 &&
                   msg->data.size() >= static_cast<size_t>(msg->height) * static_cast<size_t>(msg->step)) {
            cv::Mat raw(msg->height, msg->width, CV_8UC3,
                        const_cast<unsigned char *>(msg->data.data()), msg->step);
            std::string encoding = msg->encoding;
            std::transform(encoding.begin(), encoding.end(), encoding.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (encoding.find("rgb") != std::string::npos) {
                cv::cvtColor(raw, gray, cv::COLOR_RGB2GRAY);
            } else {
                cv::cvtColor(raw, gray, cv::COLOR_BGR2GRAY);
            }
        } else {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "visual_odometry_node image decode fallback failed (encoding=%s, step=%u, width=%u)",
                                  msg->encoding.c_str(), msg->step, msg->width);
            return;
        }
    }

    if (gray.empty()) {
        return;
    }

    const rclcpp::Time stamp = msg->header.stamp;
    int detected_features = 0;
    int tracked_features = 0;
    int inlier_features = 0;
    bool tracking_ok = false;
    std::vector<cv::Point2f> overlay_points;

    if (!initialized_) {
        resetTracking(gray, stamp);
        detected_features = static_cast<int>(prev_points_.size());
        overlay_points = prev_points_;
        publishOdometry(stamp);
        publishDebugOutputs(stamp, gray, detected_features, tracked_features, inlier_features, tracking_ok, overlay_points);
        return;
    }

    if (prev_points_.size() < static_cast<size_t>(min_tracked_features_)) {
        detectFeatures(prev_gray_, prev_points_);
    }
    detected_features = static_cast<int>(prev_points_.size());

    if (prev_points_.size() < 5) {
        resetTracking(gray, stamp);
        detected_features = static_cast<int>(prev_points_.size());
        overlay_points = prev_points_;
        publishOdometry(stamp);
        publishDebugOutputs(stamp, gray, detected_features, tracked_features, inlier_features, tracking_ok, overlay_points);
        return;
    }

    std::vector<cv::Point2f> tracked_curr;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_gray_, gray, prev_points_, tracked_curr, status, err, cv::Size(21, 21), 3);

    std::vector<cv::Point2f> matched_prev;
    std::vector<cv::Point2f> matched_curr;
    matched_prev.reserve(prev_points_.size());
    matched_curr.reserve(prev_points_.size());

    for (size_t i = 0; i < status.size(); ++i) {
        if (!status[i]) {
            continue;
        }
        if (tracked_curr[i].x < 0 || tracked_curr[i].y < 0 || tracked_curr[i].x >= gray.cols ||
            tracked_curr[i].y >= gray.rows) {
            continue;
        }
        matched_prev.push_back(prev_points_[i]);
        matched_curr.push_back(tracked_curr[i]);
    }
    tracked_features = static_cast<int>(matched_curr.size());
    overlay_points = matched_curr;

    if (matched_prev.size() < 8) {
        lost_frames_++;
        if (lost_frames_ >= max_lost_frames_) {
            resetTracking(gray, stamp);
            detected_features = static_cast<int>(prev_points_.size());
            overlay_points = prev_points_;
            publishOdometry(stamp);
            publishDebugOutputs(
                stamp, gray, detected_features, tracked_features, inlier_features, tracking_ok, overlay_points);
            return;
        }
        prev_gray_ = gray.clone();
        prev_points_ = matched_curr;
        publishOdometry(stamp);
        publishDebugOutputs(stamp, gray, detected_features, tracked_features, inlier_features, tracking_ok, overlay_points);
        return;
    }

    cv::Mat inlier_mask;
    cv::Mat essential = cv::findEssentialMat(
        matched_prev, matched_curr, K_, cv::RANSAC, 0.999, ransac_threshold_px_, inlier_mask);

    if (essential.empty()) {
        lost_frames_++;
        if (lost_frames_ >= max_lost_frames_) {
            resetTracking(gray, stamp);
            detected_features = static_cast<int>(prev_points_.size());
            overlay_points = prev_points_;
        } else {
            prev_gray_ = gray.clone();
            prev_points_ = matched_curr;
        }
        publishOdometry(stamp);
        publishDebugOutputs(stamp, gray, detected_features, tracked_features, inlier_features, tracking_ok, overlay_points);
        return;
    }

    cv::Mat R;
    cv::Mat t;
    const int inliers = cv::recoverPose(essential, matched_prev, matched_curr, K_, R, t, inlier_mask);
    inlier_features = inliers;

    if (inliers < min_inliers_) {
        lost_frames_++;
        if (lost_frames_ >= max_lost_frames_) {
            resetTracking(gray, stamp);
            detected_features = static_cast<int>(prev_points_.size());
            overlay_points = prev_points_;
        } else {
            prev_gray_ = gray.clone();
            prev_points_ = matched_curr;
        }
        publishOdometry(stamp);
        publishDebugOutputs(stamp, gray, detected_features, tracked_features, inlier_features, tracking_ok, overlay_points);
        return;
    }

    Eigen::Matrix3d R_rel;
    cv::cv2eigen(R, R_rel);
    Eigen::Quaterniond q_rel(R_rel);
    q_rel.normalize();

    Eigen::Vector3d t_rel(t.at<double>(0), t.at<double>(1), t.at<double>(2));
    if (t_rel.norm() > 1e-9) {
        t_rel.normalize();
    }
    t_rel *= translation_scale_;

    const Eigen::Quaterniond q_prev = q_w_curr_;
    t_w_curr_ = t_w_curr_ + q_prev * t_rel;
    q_w_curr_ = q_prev * q_rel;
    q_w_curr_.normalize();

    std::vector<cv::Point2f> inlier_curr_points;
    inlier_curr_points.reserve(matched_curr.size());
    for (int i = 0; i < inlier_mask.rows; ++i) {
        if (inlier_mask.at<uchar>(i) != 0) {
            inlier_curr_points.push_back(matched_curr[static_cast<size_t>(i)]);
        }
    }
    if (inlier_curr_points.size() < static_cast<size_t>(min_tracked_features_)) {
        detectFeatures(gray, inlier_curr_points);
    }
    overlay_points = inlier_curr_points;

    prev_gray_ = gray.clone();
    prev_points_ = inlier_curr_points;
    prev_stamp_ = stamp;
    lost_frames_ = 0;
    tracking_ok = true;

    publishOdometry(stamp);
    publishDebugOutputs(stamp, gray, detected_features, tracked_features, inlier_features, tracking_ok, overlay_points);
}

} // namespace super_odometry
