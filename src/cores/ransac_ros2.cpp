#include "ransac/ransac_ros2.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace m_ransac{

    RANSAC::RANSAC(std::string node_name) : rclcpp::Node(node_name){
        RCLCPP_INFO(this->get_logger(), "RANSAC node Started !!");

        this->declare_parameter("point_topic", "/ransac/point_data");
        this->declare_parameter("success_probability", 0.99);
        this->declare_parameter("outlier_ratio", 0.3);
        this->declare_parameter("subset_size", 3);
        this->declare_parameter("threshold", 1.0);
        this->declare_parameter("iteration", 0);

        LoadParameters();

        tf_broadcaster_pose_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        point_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>(point_topic_name_,10);
        inlier_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/ransac/inlier",10);
        graph_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/ransac/graph",10);
        
        callback_timer_pub_ = this->create_wall_timer(1s, std::bind(&RANSAC::PointPublish, this));
    }

    void RANSAC::LoadParameters(){
        RCLCPP_INFO(this->get_logger(),"Loading Parameters");

        this->get_parameter("point_topic", point_topic_name_);
        this->get_parameter("success_probability", s_prob_);
        this->get_parameter("outlier_ratio", out_ratio_);
        this->get_parameter("subset_size", sub_size_);
        this->get_parameter("threshold", th_);
        this->get_parameter("iteration", iter_);

        iter_ = (int)round(log(1-s_prob_)/log(1-pow(1-out_ratio_,sub_size_)));

        RCLCPP_INFO_ONCE(this->get_logger(), "The RANSAC Iteration is %d", iter_);
    }

    void RANSAC::DataGeneration(){

        std::random_device rdd;
        std::mt19937 gen(rdd());
        std::normal_distribution<float> dist(0,0.3);

        geometry_msgs::msg::Point32 point_;
        sensor_msgs::msg::ChannelFloat32 chan_;

        data_.header.stamp = this->get_clock()->now();
        data_.header.frame_id = "point";
        
        if(sub_size_== 3.0)
        {
            for(float i=-5; i<5; i+=0.1){

                point_.x = i;
                // point_.y = 0.3*i*i;// + dist(gen); 
                point_.y = 0.3*i*i + dist(gen); 
                point_.z = 0.0;

                data_.points.push_back(point_);
            }
        }   
        
        Run(data_);
        GraphPublish();
    }

    void RANSAC::Run(const sensor_msgs::msg::PointCloud pt_data_){

        int data_len = pt_data_.points.size(); // 101
        int cnt = 0; int max = 0;

        MatXd H_all(data_len,3);
        VecXd y_all(data_len,1);
        VecXd err(data_len,1);
        VecXd fit_model(data_len,1);

        for(int i=0; i<data_len; i++){
            H_all(i,0) = pow(pt_data_.points.at(i).x, 2.0);
            H_all(i,1) = pt_data_.points.at(i).x;
            H_all(i,2) = 1.0;
            y_all(i,0) = pt_data_.points.at(i).y;
            // RCLCPP_INFO(this->get_logger(), "%.2f %.2f %.2f", H_all(i,0), H_all(i,1),H_all(i,2));
        }

        std::vector<uint> idx(data_len);
        std::vector<uint> inlier_idx(data_len);
        
        for (int k=0; k<data_len; k++) idx[k] = k;

        std::random_device rd;
        std::mt19937 g(rd());

        VecXd true_coef(sub_size_,1);

        for (int k=0; k<iter_; k++){
            std::shuffle(idx.begin(), idx.end(), g);
            MatXd samp_data(sub_size_,3);

            VecXd true_out(sub_size_,1);
            VecXd est_coef(sub_size_,1);

            for (int kk=0; kk<sub_size_; kk++){
                samp_data.row(kk) = H_all.row(idx.at(kk));
                true_out.row(kk) = y_all.row(idx.at(kk));
            }
            
            est_coef = (samp_data.transpose() * samp_data).inverse() * samp_data.transpose() * true_out;

            err = ((H_all * est_coef) - y_all).array().abs();

            sensor_msgs::msg::PointCloud save_points;

            for (int l=0; l<data_len; l++){
                if(err(l,0) < th_) {
                    cnt++;

                    geometry_msgs::msg::Point32 in_point_;
        
                    in_point_.x = pt_data_.points.at(l).x;
                    in_point_.y = pt_data_.points.at(l).y;
                    in_point_.z = pt_data_.points.at(l).z;
    
                    save_points.points.push_back(in_point_);
                }
            }

            if(cnt > max){
                max = cnt;
                true_coef = est_coef;
                inlier_data_ = save_points;
            }
            cnt = 0;
        }
        RCLCPP_INFO(this->get_logger(), "%f %f %f", true_coef(0), true_coef(1), true_coef(2));
        
        fit_model = H_all*true_coef;
        for(int f=0; f<data_len; f++){
            geometry_msgs::msg::PoseStamped pose_;
            
            pose_.pose.position.x = pt_data_.points.at(f).x;
            pose_.pose.position.y = fit_model(f,0);
            pose_.pose.position.z = 0.0;

            graph_data_.poses.push_back(pose_);
        }  

        InlierPublish();  
        GraphPublish();
    }   

    void RANSAC::InlierPublish(){
        inlier_data_.header.stamp = this->get_clock()->now();
        inlier_data_.header.frame_id = "point";

        inlier_publisher_->publish(*ptr_inlier_data_);
    }

    void RANSAC::GraphPublish(){
        graph_data_.header.stamp = this->get_clock()->now();
        graph_data_.header.frame_id = "world";

        graph_publisher_->publish(*ptr_graph_data_);
    }

    void RANSAC::PointPublish(){

        DataGeneration();

        rclcpp::Time stamp_now_ = this->get_clock()->now();

        geometry_msgs::msg::TransformStamped pose_transform_;

        pose_transform_.header.stamp = stamp_now_;
        pose_transform_.header.frame_id = "world";
        pose_transform_.child_frame_id = "point";

        pose_transform_.transform.translation.x = 0.0;
        pose_transform_.transform.translation.y = 0.0;
        pose_transform_.transform.translation.z = 0.0;

        pose_transform_.transform.rotation.x = 0.0;
        pose_transform_.transform.rotation.y = 0.0;
        pose_transform_.transform.rotation.z = 0.0;
        pose_transform_.transform.rotation.w = 1.0;

        tf_broadcaster_pose_->sendTransform(pose_transform_);

        point_publisher_->publish(*ptr_data_);
    
        data_.points.clear();
        inlier_data_.points.clear();
        graph_data_.poses.clear();
    }
}