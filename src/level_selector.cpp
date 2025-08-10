#include "level_selector.hpp"
#include <fstream>
#include <sstream>

void LevelSelector::Draw() {
    if (!is_active) return;
    
    // Draw semi-transparent overlay
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), (Color){0, 0, 0, 180});
    
    // Draw menu background
    float menu_width = 600.0f;
    float menu_height = 400.0f;
    float menu_x = (GetScreenWidth() - menu_width) / 2.0f;
    float menu_y = (GetScreenHeight() - menu_height) / 2.0f;
    
    DrawRectangleRounded((Rectangle){menu_x, menu_y, menu_width, menu_height}, 0.1f, 16, LIGHTGRAY);
    DrawRectangleRoundedLines((Rectangle){menu_x, menu_y, menu_width, menu_height}, 0.1f, 16, BLACK);
    
    // Draw title
    DrawText("LEVEL SELECT", (int)(menu_x + menu_width/2 - MeasureText("LEVEL SELECT", 30)/2), (int)(menu_y + 20), 30, BLACK);
    DrawText("Press M to close", (int)(menu_x + menu_width/2 - MeasureText("Press M to close", 16)/2), (int)(menu_y + 60), 16, DARKGRAY);
    
    // Draw current level info
    DrawText(TextFormat("Current: Level %d-%d", selected_level, selected_variant), (int)(menu_x + 20), (int)(menu_y + 80), 16, DARKBLUE);
    
    // Draw level grid
    float button_width = 80.0f;
    float button_height = 40.0f;
    float start_y = menu_y + 100.0f;
    float level_spacing = 60.0f;
    float variant_spacing = 100.0f;
    
    for (int level = 1; level <= max_levels; level++) {
        float row_y = start_y + (level - 1) * level_spacing;
        
        // Draw level label
        DrawText(TextFormat("Level %d", level), (int)(menu_x + 50), (int)(row_y + 10), 20, BLACK);
        
        // Draw variant buttons
        for (int variant = 1; variant <= max_variants_per_level; variant++) {
            float button_x = menu_x + 150.0f + (variant - 1) * variant_spacing;
            Rectangle button_rect = {button_x, row_y, button_width, button_height};
            
            bool is_selected = (level == selected_level && variant == selected_variant);
            DrawLevelButton(level, variant, button_rect, is_selected);
        }
    }
    
    // Draw instructions
    const char* instruction_text = "Use arrow keys to navigate, ENTER to load level";
    DrawText(instruction_text, 
             (int)(menu_x + menu_width/2 - MeasureText(instruction_text, 14)/2), 
             (int)(menu_y + menu_height - 40), 14, DARKGRAY);
}

void LevelSelector::Update() {
    if (!is_active) return;
    
    // Navigation
    if (IsKeyPressed(KEY_LEFT)) {
        selected_variant--;
        if (selected_variant < 1) selected_variant = max_variants_per_level;
    }
    if (IsKeyPressed(KEY_RIGHT)) {
        selected_variant++;
        if (selected_variant > max_variants_per_level) selected_variant = 1;
    }
    if (IsKeyPressed(KEY_UP)) {
        selected_level--;
        if (selected_level < 1) selected_level = max_levels;
    }
    if (IsKeyPressed(KEY_DOWN)) {
        selected_level++;
        if (selected_level > max_levels) selected_level = 1;
    }
}


void LevelSelector::UpdateSequentialTransition(float dt, std::vector<Platform>& platforms, Box& box, Gorilla& gorilla) {
    if (!is_transitioning) return;
    
    switch (current_transition_state) {
        case TransitionState::IDLE:
            // Should not happen during transition, but handle it gracefully
            std::cout << "Warning: IDLE state during transition - resetting" << std::endl;
            is_transitioning = false;
            break;
            
        case TransitionState::LOADING_NEW_PLATFORMS:
            // Step 1: Load new platforms off-screen (instant)
            std::cout << "State: LOADING_NEW_PLATFORMS" << std::endl;
            platforms.clear();
            
            // Add all old platforms (still in original positions)
            for (auto& transition : old_platforms_queue) {
                platforms.push_back(transition.platform);
            }
            
            // Add all new platforms (off-screen to the right)
            for (auto& transition : new_platforms_queue) {
                Platform new_platform = transition.platform;
                new_platform.position = transition.start_position; // Start off-screen
                platforms.push_back(new_platform);
            }
            
            current_transition_state = TransitionState::MOVING_OUT_OLD_PLATFORMS;
            std::cout << "Loaded " << platforms.size() << " platforms total. Moving to MOVING_OUT_OLD_PLATFORMS" << std::endl;
            break;
            
        case TransitionState::MOVING_OUT_OLD_PLATFORMS:
            UpdateOldPlatformMovement(dt, platforms);
            break;
            
        case TransitionState::MOVING_IN_NEW_PLATFORMS:
            UpdateNewPlatformMovement(dt, platforms);
            break;
            
        case TransitionState::MOVING_BOX_AND_GORILLA:
            UpdateEntityMovement(dt, box, gorilla);
            break;
            
        case TransitionState::CLEANUP_COMPLETE:
            // Step 5: Finalize and clean up
            std::cout << "State: CLEANUP_COMPLETE" << std::endl;
            platforms.clear();
            
            // Only add final new platforms
            for (auto& transition : new_platforms_queue) {
                Platform final_platform = transition.platform;
                final_platform.position = transition.target_position;
                platforms.push_back(final_platform);
            }
            
            // Reassign platform IDs
            for (size_t i = 0; i < platforms.size(); i++) {
                platforms[i].id = (int)i;
            }
            
            // Set final positions
            box.position = box_transition.target_position;
            box.origin_position = box_transition.target_position;
            gorilla.position = gorilla_transition.target_position;
            
            // Reset state
            is_transitioning = false;
            current_transition_state = TransitionState::IDLE;
            
            std::cout << "Sequential transition complete! Final platforms: " << platforms.size() << std::endl;
            break;
    }
}

void LevelSelector::StartSequentialTransition(const std::vector<Platform>& old_platforms, const std::vector<Platform>& new_platforms, 
    const Vector2& old_box_pos, const Vector2& new_box_pos,
    const Vector2& old_gorilla_pos, const Vector2& new_gorilla_pos) {
    
    std::cout << "\n=== STARTING SEQUENTIAL TRANSITION ===" << std::endl;
    
    current_transition_state = TransitionState::LOADING_NEW_PLATFORMS;
    old_platforms_queue.clear();
    new_platforms_queue.clear();
    current_old_platform_index = 0;
    current_new_platform_index = 0;
    entity_move_timer = 0.0f;
    
    // RESET DELAY TIMERS
    old_platform_delay_timer = 0.0f;
    new_platform_delay_timer = 0.0f;
    
    // Set up old platforms queue (move left off-screen)
    for (const auto& platform : old_platforms) {
        Vector2 target_pos = {platform.position.x - 2500.0f, platform.position.y};
        old_platforms_queue.emplace_back(platform, platform.position, target_pos, true, PLATFORM_MOVE_DURATION);
        std::cout << "Queued OLD platform for exit: (" << platform.position.x << ", " << platform.position.y << ")" << std::endl;
    }
    
    // Set up new platforms queue (start from right, move to final position)
    for (const auto& platform : new_platforms) {
        Vector2 start_pos = {platform.position.x + 2500.0f, platform.position.y};
        new_platforms_queue.emplace_back(platform, start_pos, platform.position, false, PLATFORM_MOVE_DURATION);
        std::cout << "Queued NEW platform for entry: (" << start_pos.x << ", " << start_pos.y 
                  << ") -> (" << platform.position.x << ", " << platform.position.y << ")" << std::endl;
    }
    
    // Set up entity transitions
    box_transition = {old_box_pos, new_box_pos};
    gorilla_transition = {old_gorilla_pos, new_gorilla_pos};
    
    std::cout << "Box transition: (" << old_box_pos.x << ", " << old_box_pos.y 
              << ") -> (" << new_box_pos.x << ", " << new_box_pos.y << ")" << std::endl;
    std::cout << "Gorilla transition: (" << old_gorilla_pos.x << ", " << old_gorilla_pos.y 
              << ") -> (" << new_gorilla_pos.x << ", " << new_gorilla_pos.y << ")" << std::endl;
    
    is_transitioning = true;
    std::cout << "Sequential transition started!" << std::endl;
}

void LevelSelector::UpdateOldPlatformMovement(float dt, std::vector<Platform>& platforms) {
    if (current_old_platform_index >= old_platforms_queue.size()) {
        // All old platforms have finished moving
        std::cout << "All old platforms moved out. Moving to MOVING_IN_NEW_PLATFORMS" << std::endl;
        current_transition_state = TransitionState::MOVING_IN_NEW_PLATFORMS;
        return;
    }
    
    auto& current_transition = old_platforms_queue[current_old_platform_index];
    
    if (current_transition.move_state == PlatformMoveState::WAITING) {
        // Start moving this platform
        current_transition.move_state = PlatformMoveState::MOVING;
        current_transition.timer = 0.0f;
        std::cout << "Started moving OLD platform " << current_old_platform_index << std::endl;
    }
    else if (current_transition.move_state == PlatformMoveState::MOVING) {
        current_transition.timer += dt;
        float progress = current_transition.timer / current_transition.duration;
        progress = EaseOutCubic(fminf(progress, 1.0f));
        
        // Update the platform in the platforms array
        if (current_old_platform_index < platforms.size()) {
            Vector2 position_diff = Vector2Subtract(current_transition.target_position, current_transition.start_position);
            platforms[current_old_platform_index].position = Vector2Add(current_transition.start_position, Vector2Scale(position_diff, progress));
        }
        
        if (progress >= 1.0f) {
            current_transition.move_state = PlatformMoveState::COMPLETED;
            old_platform_delay_timer = 0.0f; // Start the delay timer
            std::cout << "Completed moving OLD platform " << current_old_platform_index << std::endl;
        }
    }
    else if (current_transition.move_state == PlatformMoveState::COMPLETED) {
        // Wait for delay before moving to next platform
        old_platform_delay_timer += dt;
        std::cout << "Waiting for delay: " << old_platform_delay_timer << " / " << PLATFORM_DELAY << std::endl;
        
        if (old_platform_delay_timer >= PLATFORM_DELAY) {
            current_old_platform_index++;
            old_platform_delay_timer = 0.0f;
            std::cout << "Moving to next OLD platform. Index now: " << current_old_platform_index << std::endl;
        }
    }
}

void LevelSelector::UpdateNewPlatformMovement(float dt, std::vector<Platform>& platforms) {
    if (current_new_platform_index >= new_platforms_queue.size()) {
        // All new platforms have finished moving
        std::cout << "All new platforms moved in. Moving to MOVING_BOX_AND_GORILLA" << std::endl;
        current_transition_state = TransitionState::MOVING_BOX_AND_GORILLA;
        return;
    }
    
    auto& current_transition = new_platforms_queue[current_new_platform_index];
    
    if (current_transition.move_state == PlatformMoveState::WAITING) {
        // Start moving this platform
        current_transition.move_state = PlatformMoveState::MOVING;
        current_transition.timer = 0.0f;
        std::cout << "Started moving NEW platform " << current_new_platform_index << std::endl;
    }
    else if (current_transition.move_state == PlatformMoveState::MOVING) {
        current_transition.timer += dt;
        float progress = current_transition.timer / current_transition.duration;
        progress = EaseOutCubic(fminf(progress, 1.0f));
        
        // Update the platform in the platforms array (new platforms start after old ones)
        size_t platform_index = old_platforms_queue.size() + current_new_platform_index;
        if (platform_index < platforms.size()) {
            Vector2 position_diff = Vector2Subtract(current_transition.target_position, current_transition.start_position);
            platforms[platform_index].position = Vector2Add(current_transition.start_position, Vector2Scale(position_diff, progress));
        }
        
        if (progress >= 1.0f) {
            current_transition.move_state = PlatformMoveState::COMPLETED;
            new_platform_delay_timer = 0.0f; // Start the delay timer
            std::cout << "Completed moving NEW platform " << current_new_platform_index << std::endl;
        }
    }
    else if (current_transition.move_state == PlatformMoveState::COMPLETED) {
        // Wait for delay before moving to next platform
        new_platform_delay_timer += dt;
        std::cout << "Waiting for delay: " << new_platform_delay_timer << " / " << PLATFORM_DELAY << std::endl;
        
        if (new_platform_delay_timer >= PLATFORM_DELAY) {
            current_new_platform_index++;
            new_platform_delay_timer = 0.0f;
            std::cout << "Moving to next NEW platform. Index now: " << current_new_platform_index << std::endl;
        }
    }
}

void LevelSelector::UpdateEntityMovement(float dt, Box& box, Gorilla& gorilla) {
    entity_move_timer += dt;
    float progress = entity_move_timer / ENTITY_MOVE_DURATION;
    progress = EaseOutCubic(fminf(progress, 1.0f));
    
    // Move box and gorilla simultaneously
    Vector2 box_diff = Vector2Subtract(box_transition.target_position, box_transition.start_position);
    box.position = Vector2Add(box_transition.start_position, Vector2Scale(box_diff, progress));
    
    Vector2 gorilla_diff = Vector2Subtract(gorilla_transition.target_position, gorilla_transition.start_position);
    gorilla.position = Vector2Add(gorilla_transition.start_position, Vector2Scale(gorilla_diff, progress));
    
    if (progress >= 1.0f) {
        std::cout << "Box and Gorilla movement complete. Moving to CLEANUP_COMPLETE" << std::endl;
        current_transition_state = TransitionState::CLEANUP_COMPLETE;
    }
}

void LevelSelector::DrawLevelButton(int level, int variant, Rectangle button_rect, bool is_selected) {
    // Check if this level variant actually exists
    std::string level_path = GetLevelPath(level, variant);
    bool exists = std::filesystem::exists(level_path);
    
    Color button_color = GRAY;
    Color text_color = DARKGRAY;
    
    if (exists) {
        button_color = is_selected ? DARKGREEN : GREEN;
        text_color = WHITE;
    }
    
    DrawRectangleRounded(button_rect, 0.2f, 8, button_color);
    DrawRectangleRoundedLines(button_rect, 0.2f, 8, is_selected ? YELLOW : BLACK);
    
    // Draw variant number
    const char* text = TextFormat("%d", variant);
    int text_width = MeasureText(text, 20);
    DrawText(text, 
             (int)(button_rect.x + (button_rect.width - text_width) / 2),
             (int)(button_rect.y + (button_rect.height - 20) / 2),
             20, text_color);
    
    // Draw "missing" indicator if file doesn't exist
    if (!exists) {
        DrawText("X", (int)(button_rect.x + 5), (int)(button_rect.y + 5), 12, RED);
    }
}

std::string LevelSelector::GetLevelPath(int level, int variant) {
    // Build path based on your directory structure
    std::string path = "../configs/lvl_" + std::to_string(level) + "/platform_config_" + std::to_string(variant) + ".txt";
    return path;
}

void LevelSelector::LoadNewLevel(std::vector<Platform>& platforms, Box& box, Gorilla& gorilla) {
    int original_level = selected_level;
    int original_variant = selected_variant;
    
    // Try to load next variant first
    selected_variant++;
    
    // If we've exceeded variants for this level, move to next level
    if (selected_variant > max_variants_per_level) {
        selected_variant = 1;
        selected_level++;
        
        // If we've exceeded all levels, wrap around to level 1
        if (selected_level > max_levels) {
            selected_level = 1;
        }
    }
    
    // Try to load the new level
    std::string level_path = GetLevelPath(selected_level, selected_variant);
    
    // If the file doesn't exist, try to find the next available level
    int attempts = 0;
    while (!std::filesystem::exists(level_path) && attempts < (max_levels * max_variants_per_level)) {
        selected_variant++;
        if (selected_variant > max_variants_per_level) {
            selected_variant = 1;
            selected_level++;
            if (selected_level > max_levels) {
                selected_level = 1;
            }
        }
        level_path = GetLevelPath(selected_level, selected_variant);
        attempts++;
    }
    
    // If we still can't find a valid level, revert to original
    if (!std::filesystem::exists(level_path)) {
        std::cout << "Warning: Could not find next level, staying at current level" << std::endl;
        selected_level = original_level;
        selected_variant = original_variant;
        return;
    }
    
    // Load the new level data
    std::vector<Platform> new_platforms;
    Vector2 new_box_pos, new_gorilla_pos;
    
    if (LoadLevelConfigForTransition(level_path, new_platforms, new_box_pos, new_gorilla_pos)) {
        // Start the sequential transition
        StartSequentialTransition(platforms, new_platforms, box.position, new_box_pos, gorilla.position, new_gorilla_pos);
        std::cout << "Starting sequential transition to Level " << selected_level << "-" << selected_variant << std::endl;
    } else {
        std::cout << "Failed to load Level " << selected_level << "-" << selected_variant 
                  << ", reverting to previous level" << std::endl;
        selected_level = original_level;
        selected_variant = original_variant;
    }
}

bool LevelSelector::LoadSelectedLevel(std::vector<Platform>& platforms, Box& box, Gorilla& gorilla) {
    std::string level_path = GetLevelPath(selected_level, selected_variant);
    
    if (level_path.empty() || !std::filesystem::exists(level_path)) {
        std::cout << "Level " << selected_level << "-" << selected_variant << " does not exist at: " << level_path << std::endl;
        return false;
    }
    
    return LoadLevelConfig(level_path, platforms, box, gorilla);
}

bool LevelSelector::LoadLevelConfigForTransition(const std::string& filepath, std::vector<Platform>& new_platforms, Vector2& new_box_pos, Vector2& new_gorilla_pos) {
    std::ifstream file(filepath);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filepath << std::endl;
        return false;
    }
    
    new_platforms.clear();
    
    std::string line;
    int platform_count = 0;
    bool box_position_loaded = false;
    bool gorilla_position_loaded = false;
    Vector2 default_box_position = { 75.0f, (float)GetScreenHeight() / 2.0f - 200.0f };
    Vector2 default_gorilla_position = { (float)GetScreenWidth() - 150.0f, 480.0f };
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        if (line.find("BOX") == 0) {
            float box_x, box_y;
            size_t first_comma = line.find(',');
            if (first_comma != std::string::npos) {
                std::string coords = line.substr(first_comma + 1);
                if (sscanf(coords.c_str(), " %f, %f", &box_x, &box_y) == 2) {
                    new_box_pos = { box_x, box_y };
                    box_position_loaded = true;
                }
            }
        }
        else if (line.find("GORILLA") == 0) {
            float gorilla_x, gorilla_y;
            size_t first_comma = line.find(',');
            if (first_comma != std::string::npos) {
                std::string coords = line.substr(first_comma + 1);
                if (sscanf(coords.c_str(), " %f, %f", &gorilla_x, &gorilla_y) == 2) {
                    new_gorilla_pos = { gorilla_x, gorilla_y };
                    gorilla_position_loaded = true;
                }
            }
        }
        else {
            char type;
            float center_x, center_y, width, height, rotation;
            
            if (sscanf(line.c_str(), "%c, %f, %f, %f, %f, %f", 
                       &type, &center_x, &center_y, &width, &height, &rotation) == 6) {
                if (type == 'R') {
                    Vector2 pos = { center_x, center_y };
                    Vector2 size = { width, height };
                    
                    Platform new_platform(pos, size, rotation);
                    new_platform.id = platform_count;
                    new_platforms.push_back(new_platform);
                    platform_count++;
                }
            }
        }
    }
    
    if (!box_position_loaded) {
        new_box_pos = default_box_position;
    }
    
    if (!gorilla_position_loaded) {
        new_gorilla_pos = default_gorilla_position;
    }
    
    file.close();
    return true;
}

bool LevelSelector::LoadLevelConfig(const std::string& filepath, std::vector<Platform>& platforms, Box& box, Gorilla& gorilla) {
    std::ifstream file(filepath);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filepath << std::endl;
        return false;
    }
    
    platforms.clear();
    
    std::string line;
    int platform_count = 0;
    bool box_position_loaded = false;
    bool gorilla_position_loaded = false;
    Vector2 default_box_position = { 75.0f, (float)GetScreenHeight() / 2.0f - 200.0f };
    Vector2 default_gorilla_position = { (float)GetScreenWidth() - 150.0f, 480.0f };
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        if (line.find("BOX") == 0) {
            float box_x, box_y;
            size_t first_comma = line.find(',');
            if (first_comma != std::string::npos) {
                std::string coords = line.substr(first_comma + 1);
                if (sscanf(coords.c_str(), " %f, %f", &box_x, &box_y) == 2) {
                    Vector2 box_pos = { box_x, box_y };
                    box.position = box_pos;
                    box.origin_position = box_pos;
                    box_position_loaded = true;
                }
            }
        }
        else if (line.find("GORILLA") == 0) {
            float gorilla_x, gorilla_y;
            size_t first_comma = line.find(',');
            if (first_comma != std::string::npos) {
                std::string coords = line.substr(first_comma + 1);
                if (sscanf(coords.c_str(), " %f, %f", &gorilla_x, &gorilla_y) == 2) {
                    Vector2 gorilla_pos = { gorilla_x, gorilla_y };
                    gorilla.position = gorilla_pos;
                    gorilla_position_loaded = true;
                }
            }
        }
        else {
            char type;
            float center_x, center_y, width, height, rotation;
            
            if (sscanf(line.c_str(), "%c, %f, %f, %f, %f, %f", 
                       &type, &center_x, &center_y, &width, &height, &rotation) == 6) {
                if (type == 'R') {
                    Vector2 pos = { center_x, center_y };
                    Vector2 size = { width, height };
                    
                    Platform new_platform(pos, size, rotation);
                    new_platform.id = platform_count;
                    platforms.push_back(new_platform);
                    platform_count++;
                }
            }
        }
    }
    
    if (!box_position_loaded) {
        box.position = default_box_position;
        box.origin_position = default_box_position;
    }
    
    if (!gorilla_position_loaded) {
        gorilla.position = default_gorilla_position;
    }
    
    file.close();
    return true;
}

float LevelSelector::EaseOutCubic(float t) {
    return 1.0f - powf(1.0f - t, 3.0f);
}

std::vector<std::string> LevelSelector::GetAvailableFiles(int level) {
    // This function is kept for compatibility but not used in the new system
    std::vector<std::string> files;
    return files;
}