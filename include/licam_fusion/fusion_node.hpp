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
 
#ifndef FUSION_NODE_HPP_
#define FUSION_NODE_HPP_

#include <rclcpp/rclcpp.hpp> 
#include <message_filters/subscriber.h> 
#include <message_filters/synchronizer.h> 
#include <message_filters/sync_policies/approximate_time.h> 
#include <vision_msgs/msg/detection2_d_array.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp> 
#include <sensor_msgs/msg/camera_info.hpp> 
#include <tf2_ros/transform_listener.h> 
#include <tf2_ros/buffer.h> 
#include <opencv2/opencv.hpp> 
#include <vector>
#include <memory>
#include <string>

class FusionNode : public rclcpp::Node {
public:
    
    FusionNode();

private:
    
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    
    void fuzyonislemi(const vision_msgs::msg::Detection2DArray::ConstSharedPtr bbox_msg, 
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg);

   
    bool calib_hazir_; 

    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    
    typedef message_filters::sync_policies::ApproximateTime<vision_msgs::msg::Detection2DArray, sensor_msgs::msg::PointCloud2> SyncPolicy;

    
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> bbox_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
    
    
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    
    cv::Mat K_;
    
    cv::Mat R_;
    
    cv::Mat T_;
};

#endif