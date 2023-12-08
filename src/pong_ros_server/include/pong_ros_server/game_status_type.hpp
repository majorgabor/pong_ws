#include <string>

#ifndef _GAME_STATUS_HPP_
#define _GAME_STATUS_HPP_

enum class PlayerAction
{ 
    STAY = 0u,
    UP = 1u,
    DOWN = 2u
};

struct Ball
{
    int x = 0, y = 0;
};

struct Player
{
    std::string name;
    uint8_t score = 0u;
    int paddle_position_y = 0;
};

struct GameStatus
{
    Ball ball;
    Player player_1, player_2;
    bool is_game_over = true;
    
};

#endif