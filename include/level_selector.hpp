#pragma once
#include "raylib.h"
#include "platform.hpp"
#include "box.hpp"
#include "gorilla.hpp"
#include <vector>
#include <string>
#include <filesystem>
#include <iostream>

class LevelSelector {
public:
    bool is_active = false;
    int selected_level = 1;
    int selected_variant = 1;
    int max_levels = 3;
    int max_variants_per_level = 3;
    
    void Draw();
    void Update();
    bool LoadSelectedLevel(std::vector<Platform>& platforms, Box& box, Gorilla& gorilla);
    
private:
    std::vector<std::string> GetAvailableFiles(int level);
    bool LoadLevelConfig(const std::string& filepath, std::vector<Platform>& platforms, Box& box, Gorilla& gorilla);
    void DrawLevelButton(int level, int variant, Rectangle button_rect, bool is_selected);
    std::string GetLevelPath(int level, int variant);
};