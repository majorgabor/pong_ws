#include <pong_msgs/msg/game_status.hpp>
#include <pong_msgs/msg/player_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <pong_ros_server/game_status_type.hpp>
#include <pong_ros_server/pong_logic.hpp>
#include <rclcpp/rclcpp.hpp>

#ifndef _PONG_ROS_SERVER_NODE_HPP_
#define _PONG_ROS_SERVER_NODE_HPP_

class PongRosServerNode : public rclcpp::Node
{
    std::shared_ptr<PongLogic> pong_game_logic_;

    rclcpp::TimerBase::SharedPtr timer_; // timer for game status publishing
    rclcpp::Publisher<pong_msgs::msg::GameStatus>::SharedPtr game_status_pub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr new_game_request_subs_;
    rclcpp::Subscription<pong_msgs::msg::PlayerAction>::SharedPtr
        player_action_subs_;

    void playerActionCallback(const pong_msgs::msg::PlayerAction &msg);
    void newGameRequestCallback(const std_msgs::msg::Empty&);

public:
    PongRosServerNode();

    void publishGameStatus();
};

#endif