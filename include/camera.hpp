#pragma once
#include "platform.hpp"



class SimCamera {
public:
    Camera2D camera;

    int rightmost_x;
    int leftmost_x;
    int topmost_y;
    int bottommost_y;

    SimCamera();
    void Update(float dt);
    void FindBounds(std::vector<Platform> platforms);
    
};


