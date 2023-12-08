#include <pong_ros_server/pong_ros_server_node.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PongRosServerNode>());

    rclcpp::shutdown();

    return 0;
}