#include <pong_ros_server/pong_ros_server_node.hpp>

PongRosServerNode::PongRosServerNode() : Node("pong_ros_server_node")
{
    // create pong game logic
    pong_game_logic_ = std::make_shared<PongLogic>();

    // publisher for game status
    game_status_pub_ =
        create_publisher<pong_msgs::msg::GameStatus>("/game_status", 1);

    // timer t publish game status
    timer_ = create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&PongRosServerNode::publishGameStatus, this));

    // subscribe to new game request
    new_game_request_subs_ = create_subscription<std_msgs::msg::Empty>(
        "/new_game_request", 1,
        std::bind(&PongRosServerNode::newGameRequestCallback, this,
                  std::placeholders::_1));

    // subscribe to player actions
    player_action_subs_ = create_subscription<pong_msgs::msg::PlayerAction>(
        "/player_action", 10,
        std::bind(&PongRosServerNode::playerActionCallback, this,
                  std::placeholders::_1));
}

void PongRosServerNode::newGameRequestCallback(const std_msgs::msg::Empty &)
{
    pong_game_logic_->startNewGame();
}

void PongRosServerNode::playerActionCallback(
    const pong_msgs::msg::PlayerAction &msg)
{
    pong_game_logic_->registerPlayerMovement(msg.player_name,
                                             (PlayerAction)msg.action);
}

void PongRosServerNode::publishGameStatus()
{
    GameStatus game_status = pong_game_logic_->playOneTick();

    pong_msgs::msg::GameStatus game_status_ros_msgs;
    game_status_ros_msgs.game_over = game_status.is_game_over;
    game_status_ros_msgs.ball.x_position = game_status.ball.x;
    game_status_ros_msgs.ball.y_position = game_status.ball.y;
    game_status_ros_msgs.player_1.score = game_status.player_1.score;
    game_status_ros_msgs.player_1.y_position =
        game_status.player_1.paddle_position_y;
    game_status_ros_msgs.player_2.score = game_status.player_2.score;
    game_status_ros_msgs.player_2.y_position =
        game_status.player_2.paddle_position_y;

    game_status_pub_->publish(game_status_ros_msgs);
}