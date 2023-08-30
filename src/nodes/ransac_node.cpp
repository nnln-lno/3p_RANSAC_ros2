#include <rclcpp/rclcpp.hpp>

#include "ransac/ransac_ros2.hpp"

int main(int argc, char **argv){
    // setup ROS node
    rclcpp::init(argc, argv);

    auto ransac_node = std::make_shared<m_ransac::RANSAC>("ransac_node");

    try{
        RCLCPP_INFO(ransac_node->get_logger(),"Running RANSAC algorithm...");

        rclcpp::spin(ransac_node);
        rclcpp::shutdown();
        
    }catch(std::exception const& err){
        RCLCPP_ERROR(ransac_node->get_logger(), "&s", err.what());
    }

    return 0;
}