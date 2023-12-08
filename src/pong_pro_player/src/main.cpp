#include <pong_pro_player/pong_pro_player.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PongProPlayerNode>());

    rclcpp::shutdown();

    return 0;
}