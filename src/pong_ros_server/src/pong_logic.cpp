#include <cmath>
#include <pong_ros_server/pong_logic.hpp>
#include <random>

PongLogic::PongLogic()
{
}

void PongLogic::resetGameStatus()
{
    game_status_.ball.x = 0;
    game_status_.ball.y = 0;
    game_status_.is_game_over = false;
    game_status_.player_1.score = 0;
    game_status_.player_1.paddle_position_y = 0;
    game_status_.player_2.score = 0;
    game_status_.player_2.paddle_position_y = 0;
}

void PongLogic::startBallMovement()
{
    // Getting a random direction
    std::uniform_real_distribution<double> unif(-(M_PI / 3), M_PI / 3);
    std::uniform_int_distribution<int> rand_bool(0, 1);
    std::default_random_engine re{std::random_device{}()};
    double random_direction = 0.0;
    do // make sure ball is not vertical
    {
        random_direction = unif(re);
    } while (-0.2 < random_direction and random_direction < 0.2);

    ball_dx_ = ball_speed * std::cos(random_direction);
    ball_dy_ = ball_speed * std::sin(random_direction);

    if (rand_bool(re))
    {
        ball_dx_ *= -1;
        ball_dy_ *= -1;
    }
}

void PongLogic::registerPlayerMovement(const std::string &player_name,
                                       const PlayerAction action)
{
    if (player_name == "player1")
    {
        movePlayerPaddle(game_status_.player_1, action);
    }
    else if (player_name == "player2")
    {
        movePlayerPaddle(game_status_.player_2, action);
    }
}

void PongLogic::movePlayerPaddle(Player &player, PlayerAction action)
{
    if (action == PlayerAction::UP)
    {
        player.paddle_position_y += paddle_speed;

        if (player.paddle_position_y > (screen_heigh / 2 - 30))
        {
            player.paddle_position_y = screen_heigh / 2 - 30;
        }
    }
    else if (action == PlayerAction::DOWN)
    {
        player.paddle_position_y -= paddle_speed;

        if (player.paddle_position_y < -(screen_heigh / 2 - 30))
        {
            player.paddle_position_y = -(screen_heigh / 2 - 30);
        }
    }
}

void PongLogic::startNewGame()
{
    if (game_status_.is_game_over)
    {
        resetGameStatus();
        startBallMovement();
        playOneTick();
    }
}

const GameStatus &PongLogic::playOneTick()
{
    if (not game_status_.is_game_over)
    {
        // move ball
        game_status_.ball.x += ball_dx_;
        game_status_.ball.y += ball_dy_;

        // check collision up and down
        if (game_status_.ball.y > (screen_heigh / 2))
        {
            game_status_.ball.y = (screen_heigh / 2);
            ball_dy_ *= -1;
        }
        else if (game_status_.ball.y < -(screen_heigh / 2))
        {
            game_status_.ball.y = -(screen_heigh / 2);
            ball_dy_ *= -1;
        }

        // check contact with players
        if (game_status_.ball.x > (screen_width / 2 - 50) and
            game_status_.ball.x < (screen_width / 2) and
            game_status_.ball.y <
                game_status_.player_2.paddle_position_y + 60 and
            game_status_.ball.y > game_status_.player_2.paddle_position_y - 60)
        {
            game_status_.ball.x = (screen_width / 2 - 50);
            ball_dx_ *= -1;
        }
        else if (game_status_.ball.x > (screen_width / 2))
        {
            // score
            game_status_.player_1.score++;
            game_status_.ball.x = 0;
            game_status_.ball.y = 0;
            startBallMovement();
        }
        else if (game_status_.ball.x < -(screen_width / 2 - 50) and
                 game_status_.ball.x > -(screen_width / 2) and
                 game_status_.ball.y <
                     game_status_.player_1.paddle_position_y + 60 and
                 game_status_.ball.y >
                     game_status_.player_1.paddle_position_y - 60)
        {
            game_status_.ball.x = -(screen_width / 2 - 50);
            ball_dx_ *= -1;
        }
        else if (game_status_.ball.x < -(screen_width / 2))
        {
            // score
            game_status_.player_2.score++;
            game_status_.ball.x = 0;
            game_status_.ball.y = 0;
            startBallMovement();
        }

        // check if end of game
        if (game_status_.player_1.score >= 3 or
            game_status_.player_2.score >= 3)
        {
            game_status_.is_game_over = true;
        }
    }

    return game_status_;
}
