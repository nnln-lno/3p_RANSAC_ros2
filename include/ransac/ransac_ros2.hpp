#ifndef RANSAC_ROS2_HPP
#define RANSAC_ROS2_HPP

#include <queue>
#include <chrono>
#include <memory>
#include <string>
#include <random>

#include <ransac/ransac_variable.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2_ros/transform_broadcaster.h>

namespace m_ransac{

    class RANSAC : public rclcpp::Node{
        public:    
            RANSAC(std::string node_name);

        private:

            std::string point_topic_name_;

            int iter_;
            int sub_size_;

            double s_prob_;
            double out_ratio_;
            double th_;

            VecXd y_true;

            sensor_msgs::msg::PointCloud data_;
            sensor_msgs::msg::PointCloud *ptr_data_ = &data_;

            sensor_msgs::msg::PointCloud inlier_data_;
            sensor_msgs::msg::PointCloud *ptr_inlier_data_ = &inlier_data_;

            nav_msgs::msg::Path graph_data_;
            nav_msgs::msg::Path *ptr_graph_data_ = &graph_data_;

            rclcpp::TimerBase::SharedPtr callback_timer_pub_;

            rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr point_publisher_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr inlier_publisher_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr graph_publisher_;

            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_pose_;

            void LoadParameters();
            void PointPublish();
            void InlierPublish();
            void GraphPublish();
            void DataGeneration();
            void Run(const sensor_msgs::msg::PointCloud pt_data_);
    }; // Class
} // Namespace

#endif