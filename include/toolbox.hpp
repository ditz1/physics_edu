#pragma once
#include "spring.hpp"
#include "box.hpp"
#include "ball_and_string.hpp"
#include "platform.hpp"


class Toolbox {
public:
    void Draw();

    std::vector<Object*> objects; // Collection of objects to manage

};