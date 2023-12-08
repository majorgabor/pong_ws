#include <pong_pro_player/pong_pro_player.hpp>

PongProPlayerNode::PongProPlayerNode() : Node("pong_pro_player")
{
    game_status_subs_ = create_subscription<pong_msgs::msg::GameStatus>(
        "/game_status", 10,
        std::bind(&PongProPlayerNode::gameStatusUpdateCallback, this,
                  std::placeholders::_1));

    player_action_pub_ =
        create_publisher<pong_msgs::msg::PlayerAction>("/player_action", 1);
}

void PongProPlayerNode::gameStatusUpdateCallback(
    const pong_msgs::msg::GameStatus &msg)
{
    pong_msgs::msg::PlayerAction player_action;

    player_action.player_name = "player1";

    int y_diff = (msg.ball.y_position - msg.player_1.y_position);

    if (y_diff < 10 and y_diff > -10)
    {
        player_action.action = pong_msgs::msg::PlayerAction::STAY;
    }
    else if (y_diff >= 10)
    {
        player_action.action = pong_msgs::msg::PlayerAction::UP;
    }
    else
    {
        player_action.action = pong_msgs::msg::PlayerAction::DOWN;
    }

    player_action_pub_->publish(player_action);
}