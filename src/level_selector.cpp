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

// Fixed: loads the next stage in the level, if already at the end it will load the next tier of level
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
    
    // Load the level
    if (LoadLevelConfig(level_path, platforms, box, gorilla)) {
        std::cout << "Successfully loaded Level " << selected_level << "-" << selected_variant 
                  << " from: " << level_path << std::endl;
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

bool LevelSelector::LoadLevelConfigTransition(const std::string& filepath, std::vector<Platform>& platforms, Box& box, Gorilla& gorilla) {
    std::ifstream file(filepath);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filepath << std::endl;
        return false;
    }

    std::vector<Platform> old_platforms;
    Vector2 old_box_position;
    Vector2 old_gorilla_position;
    
    platforms.clear();
    
    std::string line;
    int platform_count = 0;
    bool box_position_loaded = false;
    bool gorilla_position_loaded = false;
    Vector2 default_box_position = { 75.0f, (float)GetScreenHeight() / 2.0f - 200.0f }; // Default position
    Vector2 default_gorilla_position = { (float)GetScreenWidth() - 150.0f, 480.0f }; // Default position
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        // Check if this line defines a box position
        if (line.find("BOX") == 0) {
            // Parse BOX line: "BOX, x, y"
            float box_x, box_y;
            
            // Find the first comma and parse from there
            size_t first_comma = line.find(',');
            if (first_comma != std::string::npos) {
                std::string coords = line.substr(first_comma + 1);
                
                if (sscanf(coords.c_str(), " %f, %f", &box_x, &box_y) == 2) {
                    Vector2 box_pos = { box_x, box_y };
                    box.position = box_pos;
                    box.origin_position = box_pos; // Set the origin position for reset
                    box_position_loaded = true;
                    std::cout << "Loaded box position: (" << box_x << ", " << box_y << ")" << std::endl;
                } else {
                    std::cout << "Failed to parse box coordinates from: " << coords << std::endl;
                }
            } else {
                std::cout << "Failed to find comma in BOX line: " << line << std::endl;
            }
        }
        // Check if this line defines a gorilla position
        else if (line.find("GORILLA") == 0) {
            // Parse GORILLA line: "GORILLA, x, y"
            float gorilla_x, gorilla_y;
            
            size_t first_comma = line.find(',');
            if (first_comma != std::string::npos) {
                std::string coords = line.substr(first_comma + 1);
                
                if (sscanf(coords.c_str(), " %f, %f", &gorilla_x, &gorilla_y) == 2) {
                    Vector2 gorilla_pos = { gorilla_x, gorilla_y };
                    gorilla.position = gorilla_pos;
                    gorilla_position_loaded = true;
                    std::cout << "Loaded gorilla position: (" << gorilla_x << ", " << gorilla_y << ")" << std::endl;
                } else {
                    std::cout << "Failed to parse gorilla coordinates from: " << coords << std::endl;
                }
            } else {
                std::cout << "Failed to find comma in GORILLA line: " << line << std::endl;
            }
        }
        // Otherwise check if this line defines a platform
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
                    std::cout << "Loaded platform " << platform_count << ": pos(" << pos.x << "," << pos.y 
                             << ") size(" << size.x << "," << size.y << ") rot(" << rotation << ")" << std::endl;
                }
            } else {
                std::cout << "Failed to parse line: " << line << std::endl;
            }
        }
    }

    
    // If no box position was loaded from file, use default
    if (!box_position_loaded) {
        box.position = default_box_position;
        box.origin_position = default_box_position;
        std::cout << "Using default box position: (" << default_box_position.x << ", " << default_box_position.y << ")" << std::endl;
    }
    
    // If no gorilla position was loaded from file, use default
    if (!gorilla_position_loaded) {
        gorilla.position = default_gorilla_position;
        std::cout << "Using default gorilla position: (" << default_gorilla_position.x << ", " << default_gorilla_position.y << ")" << std::endl;
    }
    
    std::cout << "Level configuration loaded from " << filepath << " - " << platform_count << " platforms loaded" << std::endl;
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
    Vector2 default_box_position = { 75.0f, (float)GetScreenHeight() / 2.0f - 200.0f }; // Default position
    Vector2 default_gorilla_position = { (float)GetScreenWidth() - 150.0f, 480.0f }; // Default position
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        // Check if this line defines a box position
        if (line.find("BOX") == 0) {
            // Parse BOX line: "BOX, x, y"
            float box_x, box_y;
            
            // Find the first comma and parse from there
            size_t first_comma = line.find(',');
            if (first_comma != std::string::npos) {
                std::string coords = line.substr(first_comma + 1);
                
                if (sscanf(coords.c_str(), " %f, %f", &box_x, &box_y) == 2) {
                    Vector2 box_pos = { box_x, box_y };
                    box.position = box_pos;
                    box.origin_position = box_pos; // Set the origin position for reset
                    box_position_loaded = true;
                    std::cout << "Loaded box position: (" << box_x << ", " << box_y << ")" << std::endl;
                } else {
                    std::cout << "Failed to parse box coordinates from: " << coords << std::endl;
                }
            } else {
                std::cout << "Failed to find comma in BOX line: " << line << std::endl;
            }
        }
        // Check if this line defines a gorilla position
        else if (line.find("GORILLA") == 0) {
            // Parse GORILLA line: "GORILLA, x, y"
            float gorilla_x, gorilla_y;
            
            size_t first_comma = line.find(',');
            if (first_comma != std::string::npos) {
                std::string coords = line.substr(first_comma + 1);
                
                if (sscanf(coords.c_str(), " %f, %f", &gorilla_x, &gorilla_y) == 2) {
                    Vector2 gorilla_pos = { gorilla_x, gorilla_y };
                    gorilla.position = gorilla_pos;
                    gorilla_position_loaded = true;
                    std::cout << "Loaded gorilla position: (" << gorilla_x << ", " << gorilla_y << ")" << std::endl;
                } else {
                    std::cout << "Failed to parse gorilla coordinates from: " << coords << std::endl;
                }
            } else {
                std::cout << "Failed to find comma in GORILLA line: " << line << std::endl;
            }
        }
        // Otherwise check if this line defines a platform
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
                    std::cout << "Loaded platform " << platform_count << ": pos(" << pos.x << "," << pos.y 
                             << ") size(" << size.x << "," << size.y << ") rot(" << rotation << ")" << std::endl;
                }
            } else {
                std::cout << "Failed to parse line: " << line << std::endl;
            }
        }
    }
    
    // If no box position was loaded from file, use default
    if (!box_position_loaded) {
        box.position = default_box_position;
        box.origin_position = default_box_position;
        std::cout << "Using default box position: (" << default_box_position.x << ", " << default_box_position.y << ")" << std::endl;
    }
    
    // If no gorilla position was loaded from file, use default
    if (!gorilla_position_loaded) {
        gorilla.position = default_gorilla_position;
        std::cout << "Using default gorilla position: (" << default_gorilla_position.x << ", " << default_gorilla_position.y << ")" << std::endl;
    }
    
    std::cout << "Level configuration loaded from " << filepath << " - " << platform_count << " platforms loaded" << std::endl;
    file.close();
    return true;
}