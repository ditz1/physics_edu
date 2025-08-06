#pragma once
#include "spring.hpp"
#include "box.hpp"
#include "ball_and_string.hpp"
#include "platform.hpp"
#include <fstream>  // need for file creation
#include <iostream>
#include <string>

extern std::vector<Platform> all_platforms;

class Toolbox {
public:
    void Draw();
    void Update(float dt, std::vector<Platform>& platforms, const Box& box);
    void SavePlatformConfiguration(const std::vector<Platform>& platforms, const Box& box);    
    std::vector<Object*> objects;
    
    // Platform creation state
    bool creating_platform = false;
    Vector2 platform_start_pos;
    Vector2 platform_current_pos;
    bool mouse_pressed = false;
};