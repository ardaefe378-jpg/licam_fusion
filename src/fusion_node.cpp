/*
 * Copyright 2026 [Arda Efe Yilmaz]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include "licam_fusion/fusion_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <algorithm>

FusionNode::FusionNode() : Node("kamera_lidar_fuzyon_node"), calib_hazir_(false) {
    
    this->declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    this->declare_parameter<std::string>("bbox_topic", "/perception/stereo_detections");
    this->declare_parameter<std::string>("lidar_topic", "/points_downsampled");
    this->declare_parameter<std::string>("lidar_frame_id", "cloud");
    this->declare_parameter<int>("sync_queue_size", 10);

    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    std::string bbox_topic = this->get_parameter("bbox_topic").as_string();
    std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
    lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();
    int sync_queue = this->get_parameter("sync_queue_size").as_int();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&FusionNode::cameraInfoCallback, this, std::placeholders::_1)
    );
    
    bbox_sub_.subscribe(this, bbox_topic, rmw_qos_profile_sensor_data);
    lidar_sub_.subscribe(this, lidar_topic, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(sync_queue), bbox_sub_, lidar_sub_);
    sync_->registerCallback(std::bind(&FusionNode::fuzyonislemi, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "LiCam Fusion Node is Ready. Lidar Frame: %s", lidar_frame_id_.c_str());
}

void FusionNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!calib_hazir_) {
        K_ = (cv::Mat_<double>(3, 3) << 
              msg->k[0], msg->k[1], msg->k[2],
              msg->k[3], msg->k[4], msg->k[5],
              msg->k[6], msg->k[7], msg->k[8]);
        
        try {
            auto transform = tf_buffer_->lookupTransform(
                msg->header.frame_id, 
                lidar_frame_id_,
                tf2::TimePointZero);

            T_ = (cv::Mat_<double>(3, 1) << 
                  transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z);

            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            
            R_ = (cv::Mat_<double>(3, 3) << 
                  m[0][0], m[0][1], m[0][2],
                  m[1][0], m[1][1], m[1][2],
                  m[2][0], m[2][1], m[2][2]);

            calib_hazir_ = true;
            RCLCPP_INFO(this->get_logger(), "Calibration matrices successfully received.");
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Waiting for TF drift : %s", ex.what());
        }
    }
}

void FusionNode::fuzyonislemi(const vision_msgs::msg::Detection2DArray::ConstSharedPtr bbox_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg) {
    if (!calib_hazir_) return;
    
    struct BBoxData {
        vision_msgs::msg::Detection2D detection;
        std::vector<double> valid_depths;
    };
    
    std::vector<BBoxData> BBoxListesi;
    for (const auto& det : bbox_msg->detections) {
        BBoxListesi.push_back({det, {}});
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*lidar_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*lidar_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*lidar_msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        cv::Mat pt_3d = (cv::Mat_<double>(3, 1) << *iter_x, *iter_y, *iter_z);
        cv::Mat pt_cam = R_ * pt_3d + T_;
        double Zc = pt_cam.at<double>(2);

        if (Zc <= 0.0) continue;
        
        cv::Mat pt_pixel = K_ * pt_cam;
        int u = static_cast<int>(pt_pixel.at<double>(0) / Zc);
        int v = static_cast<int>(pt_pixel.at<double>(1) / Zc);

        for (auto& item : BBoxListesi) {
            float cx = item.detection.bbox.center.position.x;
            float cy = item.detection.bbox.center.position.y;
            float w = item.detection.bbox.size.x;
            float h = item.detection.bbox.size.y;

            int x_min = static_cast<int>(cx - w / 2.0);
            int x_max = static_cast<int>(cx + w / 2.0);
            int y_min = static_cast<int>(cy - h / 2.0);
            int y_max = static_cast<int>(cy + h / 2.0);

            if (u >= x_min && u <= x_max && v >= y_min && v <= y_max) {
                item.valid_depths.push_back(Zc);
            }
        }
    }
    
    for (auto& item : BBoxListesi) {
        if (item.valid_depths.empty()) continue;
        std::sort(item.valid_depths.begin(), item.valid_depths.end());
        double final_depth = item.valid_depths[item.valid_depths.size() / 2];
        RCLCPP_INFO(this->get_logger(), "Obje ID: %s | Uzerine Dusen Nokta: %zu | Mesafe: %.2f m", item.detection.id.c_str(), item.valid_depths.size(), final_depth);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FusionNode>());
    rclcpp::shutdown();
    return 0;
}