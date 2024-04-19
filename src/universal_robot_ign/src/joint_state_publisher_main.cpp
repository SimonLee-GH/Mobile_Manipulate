/*******************************************************************************
 *  Copyright (c) Gezp (https://github.com/gezp), All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <universal_robot_ign/joint_state_publisher.hpp>

int main(int argc, char* argv[])
{
    // create ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("joint_state_publisher");
//     // variable
//     std::vector<std::string> joint_names;
//     std::string ign_topic;
//     int update_rate;
//     // get parameter
//     ros_node->declare_parameter("joint_names");
//     ros_node->declare_parameter("ign_topic");
//     ros_node->declare_parameter("rate", 30);
//     joint_names = ros_node->get_parameter("joint_names").as_string_array();
//     ign_topic = ros_node->get_parameter("ign_topic").as_string();
//     update_rate = ros_node->get_parameter("rate").as_int();

//     // create publisher
//     auto joint_publisher = std::make_shared<universal_robot_ign::JointStatePublisher>(ros_node,
//         joint_names, "joint_states", ign_topic,update_rate);
//     // run node until it's exited
//     rclcpp::spin(ros_node);
//     //clean up
//     rclcpp::shutdown();
//     return 0;
// }

// / Declare parameters with their default values
    ros_node->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>()); // 默认为空的字符串向量
    ros_node->declare_parameter<std::string>("ign_topic", ""); // 默认为空字符串
    ros_node->declare_parameter<int>("rate", 30); // 默认值为30

    // Get the parameters
    std::vector<std::string> joint_names = ros_node->get_parameter("joint_names").as_string_array();
    std::string ign_topic = ros_node->get_parameter("ign_topic").as_string();
    int update_rate = ros_node->get_parameter("rate").as_int();

    // Create the joint state publisher
    auto joint_publisher = std::make_shared<universal_robot_ign::JointStatePublisher>(
        ros_node, joint_names, "joint_states", ign_topic, update_rate);

    // Run the node until it's exited
    rclcpp::spin(ros_node);

    // Clean up and shut down
    rclcpp::shutdown();
    return 0;
}