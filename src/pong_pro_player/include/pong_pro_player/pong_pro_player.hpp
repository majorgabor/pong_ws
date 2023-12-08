#include <pong_msgs/msg/game_status.hpp>
#include <pong_msgs/msg/player_action.hpp>
#include <rclcpp/rclcpp.hpp>

#ifndef _PONG_PRO_PLAYER_HPP_
#define _PONG_PRO_PLAYER_HPP_

class PongProPlayerNode : public rclcpp::Node
{
    rclcpp::Publisher<pong_msgs::msg::PlayerAction>::SharedPtr
        player_action_pub_;
    rclcpp::Subscription<pong_msgs::msg::GameStatus>::SharedPtr
        game_status_subs_;

    void gameStatusUpdateCallback(const pong_msgs::msg::GameStatus &msg);

public:
    PongProPlayerNode();
};

#endif