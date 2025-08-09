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

void LevelSelector::UpdateTransition(float dt, std::vector<Platform>& platforms, Box& box, Gorilla& gorilla) {
    if (!is_transitioning) return;
    
    transition_timer += dt;
    float progress = transition_timer / TRANSITION_DURATION;
    progress = EaseInOutCubic(fminf(progress, 1.0f)); // Apply easing
    
    // Debug output every half second
    static float last_debug_time = 0.0f;
    last_debug_time += dt;
    if (last_debug_time >= 0.5f) {
        std::cout << "\n=== TRANSITION UPDATE ===" << std::endl;
        std::cout << "Progress: " << (progress * 100.0f) << "%" << std::endl;
        std::cout << "Transitioning platforms: " << transitioning_platforms.size() << std::endl;
        last_debug_time = 0.0f;
    }
    
    // CRITICAL FIX: Clear platforms and rebuild from transitioning platforms
    platforms.clear();
    
    // Process each transitioning platform
    for (size_t i = 0; i < transitioning_platforms.size(); i++) {
        auto& transition = transitioning_platforms[i];
        Platform animated_platform = transition.platform;
        
        // Calculate current position using proper interpolation
        Vector2 position_diff = Vector2Subtract(transition.target_position, transition.start_position);
        animated_platform.position = Vector2Add(transition.start_position, Vector2Scale(position_diff, progress));
        
        // Debug: Show what's happening with each platform
        if (last_debug_time == 0.0f) { // Only on debug frames
            std::cout << "Platform " << i << " (" << (transition.is_old_platform ? "OLD" : "NEW") << "): "
                      << "Start(" << transition.start_position.x << "," << transition.start_position.y << ") -> "
                      << "Target(" << transition.target_position.x << "," << transition.target_position.y << ") = "
                      << "Current(" << animated_platform.position.x << "," << animated_platform.position.y << ")" << std::endl;
        }
        
        // ALWAYS add the platform during transition - no filtering
        platforms.push_back(animated_platform);
    }
    
    // Update box position  
    Vector2 box_diff = Vector2Subtract(box_transition.target_position, box_transition.start_position);
    box.position = Vector2Add(box_transition.start_position, Vector2Scale(box_diff, progress));
    box.origin_position = box_transition.target_position; // Update origin for reset functionality
    
    // Update gorilla position
    Vector2 gorilla_diff = Vector2Subtract(gorilla_transition.target_position, gorilla_transition.start_position);
    gorilla.position = Vector2Add(gorilla_transition.start_position, Vector2Scale(gorilla_diff, progress));
    
    // Check if transition is complete
    if (transition_timer >= TRANSITION_DURATION) {
        std::cout << "\n=== TRANSITION COMPLETE ===" << std::endl;
        is_transitioning = false;
        transition_timer = 0.0f;
        
        // Finalize positions and clean up
        platforms.clear();
        for (auto& transition : transitioning_platforms) {
            if (!transition.is_old_platform) { // Only keep new platforms
                Platform final_platform = transition.platform;
                final_platform.position = transition.target_position;
                platforms.push_back(final_platform);
                std::cout << "Final platform at (" << final_platform.position.x << "," << final_platform.position.y << ")" << std::endl;
            }
        }
        
        // Reassign platform IDs
        for (size_t i = 0; i < platforms.size(); i++) {
            platforms[i].id = (int)i;
        }
        
        // Set final positions
        box.position = box_transition.target_position;
        box.origin_position = box_transition.target_position;
        gorilla.position = gorilla_transition.target_position;
        
        transitioning_platforms.clear();
        std::cout << "Level transition complete with " << platforms.size() << " final platforms!" << std::endl;
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
        // Start the animated transition
        StartTransition(platforms, new_platforms, box.position, new_box_pos, gorilla.position, new_gorilla_pos);
        std::cout << "Starting animated transition to Level " << selected_level << "-" << selected_variant << std::endl;
    } else {
        std::cout << "Failed to load Level " << selected_level << "-" << selected_variant 
                  << ", reverting to previous level" << std::endl;
        selected_level = original_level;
        selected_variant = original_variant;
    }
}

void LevelSelector::StartTransition(const std::vector<Platform>& old_platforms, const std::vector<Platform>& new_platforms, 
    const Vector2& old_box_pos, const Vector2& new_box_pos,
    const Vector2& old_gorilla_pos, const Vector2& new_gorilla_pos) {
    is_transitioning = true;
    transition_timer = 0.0f;
    transitioning_platforms.clear();
    
    std::cout << "\n=== STARTING TRANSITION ===" << std::endl;
    std::cout << "Old platforms: " << old_platforms.size() << std::endl;
    std::cout << "New platforms: " << new_platforms.size() << std::endl;
        
    // Set up old platforms to slide out to the left
    for (const auto& platform : old_platforms) {
        Vector2 target_pos = {platform.position.x - 3000.0f, platform.position.y}; // Even further left
        transitioning_platforms.emplace_back(platform, platform.position, target_pos, true);
        std::cout << "OLD Platform: (" << platform.position.x << ", " << platform.position.y 
                  << ") -> (" << target_pos.x << ", " << target_pos.y << ")" << std::endl;
    }
    
    // Set up new platforms to slide in from the right
    for (const auto& platform : new_platforms) {
        Vector2 start_pos = {platform.position.x + 3000.0f, platform.position.y}; // Even further right
        transitioning_platforms.emplace_back(platform, start_pos, platform.position, false);
        std::cout << "NEW Platform: (" << start_pos.x << ", " << start_pos.y 
                  << ") -> (" << platform.position.x << ", " << platform.position.y << ")" << std::endl;
    }
    
    // Set up box transition
    box_transition.start_position = old_box_pos;
    box_transition.target_position = new_box_pos;
    std::cout << "Box: (" << old_box_pos.x << ", " << old_box_pos.y 
              << ") -> (" << new_box_pos.x << ", " << new_box_pos.y << ")" << std::endl;
    
    // Set up gorilla transition
    gorilla_transition.start_position = old_gorilla_pos;
    gorilla_transition.target_position = new_gorilla_pos;
    std::cout << "Gorilla: (" << old_gorilla_pos.x << ", " << old_gorilla_pos.y 
              << ") -> (" << new_gorilla_pos.x << ", " << new_gorilla_pos.y << ")" << std::endl;
    
    std::cout << "Total transitioning platforms: " << transitioning_platforms.size() << std::endl;
    std::cout << "=== TRANSITION SETUP COMPLETE ===" << std::endl;
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

float LevelSelector::EaseInOutCubic(float t) {
    return t < 0.5f ? 4.0f * t * t * t : 1.0f - powf(-2.0f * t + 2.0f, 3.0f) / 2.0f;
}