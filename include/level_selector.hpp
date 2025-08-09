#pragma once
#include "raylib.h"
#include "platform.hpp"
#include "box.hpp"
#include "gorilla.hpp"
#include <vector>
#include <string>
#include <filesystem>
#include <iostream>

struct PlatformTransition {
    Platform platform;
    Vector2 start_position;
    Vector2 target_position;
    bool is_old_platform; // true for sliding out, false for sliding in
    
    // Constructor to properly initialize the Platform
    PlatformTransition(const Platform& p, Vector2 start_pos, Vector2 target_pos, bool is_old)
        : platform(p), start_position(start_pos), target_position(target_pos), is_old_platform(is_old) {}
};

struct EntityTransition {
    Vector2 start_position;
    Vector2 target_position;
};

class LevelSelector {
public:
    bool is_active = false;
    int selected_level = 1;
    int selected_variant = 1;
    int max_levels = 3;
    int max_variants_per_level = 3;
    
    // Animation state
    bool is_transitioning = false;
    float transition_timer = 0.0f;
    const float TRANSITION_DURATION = 1.5f; // Total transition time
    std::vector<PlatformTransition> transitioning_platforms;
    EntityTransition box_transition;
    EntityTransition gorilla_transition;
    
    void Draw();
    void Update();
    bool LoadSelectedLevel(std::vector<Platform>& platforms, Box& box, Gorilla& gorilla);
    void LoadNewLevel(std::vector<Platform>& platforms, Box& box, Gorilla& gorilla);
    void UpdateTransition(float dt, std::vector<Platform>& platforms, Box& box, Gorilla& gorilla);
    
private:
    std::vector<std::string> GetAvailableFiles(int level);
    bool LoadLevelConfig(const std::string& filepath, std::vector<Platform>& platforms, Box& box, Gorilla& gorilla);
    bool LoadLevelConfigForTransition(const std::string& filepath, std::vector<Platform>& new_platforms, Vector2& new_box_pos, Vector2& new_gorilla_pos);
    void StartTransition(const std::vector<Platform>& old_platforms, const std::vector<Platform>& new_platforms, 
                        const Vector2& old_box_pos, const Vector2& new_box_pos,
                        const Vector2& old_gorilla_pos, const Vector2& new_gorilla_pos);
    void DrawLevelButton(int level, int variant, Rectangle button_rect, bool is_selected);
    std::string GetLevelPath(int level, int variant);
    float EaseInOutCubic(float t); // Smooth easing function
};