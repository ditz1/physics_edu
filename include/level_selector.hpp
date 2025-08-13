#pragma once
#include "raylib.h"
#include "platform.hpp"
#include "box.hpp"
#include "gorilla.hpp"
#include <vector>
#include <string>
#include <filesystem>
#include <iostream>

enum class TransitionState {
    IDLE,
    LOADING_NEW_PLATFORMS,
    MOVING_OUT_OLD_PLATFORMS,
    MOVING_IN_NEW_PLATFORMS,
    MOVING_BOX_AND_GORILLA,
    CLEANUP_COMPLETE
};

enum class PlatformMoveState {
    WAITING,
    MOVING,
    COMPLETED
};

struct SequentialPlatformTransition {
    Platform platform;
    Vector2 start_position;
    Vector2 target_position;
    bool is_old_platform;
    PlatformMoveState move_state;
    float timer;
    float duration;
    
    SequentialPlatformTransition(const Platform& p, Vector2 start_pos, Vector2 target_pos, bool is_old, float move_duration = 0.5f)
        : platform(p), start_position(start_pos), target_position(target_pos), 
          is_old_platform(is_old), move_state(PlatformMoveState::WAITING), 
          timer(0.0f), duration(move_duration) {}
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
    
    // Sequential animation state
    bool is_transitioning = false;
    TransitionState current_transition_state = TransitionState::IDLE;
    std::vector<SequentialPlatformTransition> old_platforms_queue;
    std::vector<SequentialPlatformTransition> new_platforms_queue;
    size_t current_old_platform_index = 0;
    size_t current_new_platform_index = 0;
    
    EntityTransition box_transition;
    EntityTransition gorilla_transition;
    float entity_move_timer = 0.0f;
    
    // NEW: Add delay timers as member variables
    float old_platform_delay_timer = 0.0f;
    float new_platform_delay_timer = 0.0f;
    
    const float ENTITY_MOVE_DURATION = 0.3f;
    const float PLATFORM_MOVE_DURATION = 0.3f; // Time for each platform to move
    const float PLATFORM_DELAY = 0.05f; // Delay between platform movements
    
    void Draw();
    void Update();
    bool LoadSelectedLevel(std::vector<Platform>& platforms, Box& box, Gorilla& gorilla);
    void LoadNewLevel(std::vector<Platform>& platforms, Box& box, Gorilla& gorilla);
    void UpdateSequentialTransition(float dt, std::vector<Platform>& platforms, Box& box, Gorilla& gorilla);
    void SaveCurrentLevel(const std::vector<Platform>& platforms, const Box& box, const Gorilla& gorilla);

    
private:
    std::vector<std::string> GetAvailableFiles(int level);
    bool LoadLevelConfig(const std::string& filepath, std::vector<Platform>& platforms, Box& box, Gorilla& gorilla);
    bool LoadLevelConfigForTransition(const std::string& filepath, std::vector<Platform>& new_platforms, Vector2& new_box_pos, Vector2& new_gorilla_pos);
    void StartSequentialTransition(const std::vector<Platform>& old_platforms, const std::vector<Platform>& new_platforms, 
                                   const Vector2& old_box_pos, const Vector2& new_box_pos,
                                   const Vector2& old_gorilla_pos, const Vector2& new_gorilla_pos);
    void UpdateOldPlatformMovement(float dt, std::vector<Platform>& platforms);
    void UpdateNewPlatformMovement(float dt, std::vector<Platform>& platforms);
    void UpdateEntityMovement(float dt, Box& box, Gorilla& gorilla);
    void DrawLevelButton(int level, int variant, Rectangle button_rect, bool is_selected);
    std::string GetLevelPath(int level, int variant);
    float EaseOutCubic(float t); // Smooth easing function for individual movements
};