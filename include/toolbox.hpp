#pragma once
#include "spring.hpp"
#include "box.hpp"
#include <vector>


class Toolbox {
public:
    void Draw();

    std::vector<Object*> objects; // Collection of objects to manage

};