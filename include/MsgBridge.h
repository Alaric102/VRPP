#ifndef _MSGBRIDGE_H_
#define _MSGBRIDGE_H_

#include <string>
#include <sstream>
#include <iostream>

enum UnityCommands{
    setStartPoint = 1,
    setGoalPoint
};

struct Vector2int_t{
    int x = 0;
    int y = 0;
};

static Vector2int_t CmdSetStartPoint(std::stringstream &ss){
    Vector2int_t res = {0, 0};
    ss >> res.x;
    ss >> res.y;
    return res;
}

static Vector2int_t CmdSetGoalPoint(std::stringstream &ss){
    Vector2int_t res = {0, 0};
    ss >> res.x;
    ss >> res.y;
    return res;
}

void processString(std::string s){
    std::stringstream sStream;
    sStream.str(s);

    int type = 0;
    sStream >> type;
    switch (type) {
        case setStartPoint: {
            Vector2int_t start2D = CmdSetStartPoint(sStream);
            std::cout << start2D.x << " " << start2D.y << std::endl;
            break;
        }
        case setGoalPoint: {
            Vector2int_t goal2D = CmdSetStartPoint(sStream);
            std::cout << goal2D.x << " " << goal2D.y << std::endl;
            break;
        }
        default:
            break;
    }
}



#endif
