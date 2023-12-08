#include <memory>
#include <pong_ros_server/game_status_type.hpp>

#ifndef _PONG_LOGIC_HPP_
#define _PONG_LOGIC_HPP_

constexpr int screen_width = 1500;
constexpr int screen_heigh = 1000;
constexpr uint ball_speed = 20;
constexpr uint paddle_speed = 20;

class PongLogic
{
    int ball_dx_, ball_dy_;
    GameStatus game_status_;

    void resetGameStatus();
    void startBallMovement();
    void movePlayerPaddle(Player& player, PlayerAction action);

public:
    PongLogic();

    void registerPlayerMovement(const std::string &player_name,
                                const PlayerAction action);
    void startNewGame();
    const GameStatus &playOneTick();
};

#endif