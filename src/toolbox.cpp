#include <toolbox.hpp>

void Toolbox::Draw() {
    // draw bar on top
    float bar_start_x = GetScreenWidth() / 2.0f - 100.0f;
    float bar_height = 150.0f;
    float bar_width = GetScreenWidth() / 2.0f;
    Rectangle bar_rect = { bar_start_x, 10, bar_width - 20, bar_height };
    DrawRectangleRounded(bar_rect, 0.3f, 20, LIGHTGRAY);

    // draw selectable objects
    float button_width = 50.0f;
    float button_height = 50.0f;
    int button_start_x = GetScreenWidth() / 2.0f;
    int button_start_y = 20;
    
    // Platform creation button - highlight if in creation mode
    Color platform_button_color = creating_platform ? DARKGREEN : GREEN;
    DrawRectangle(button_start_x, button_start_y, button_width, button_height, platform_button_color);
    DrawText("1", button_start_x + 20, button_start_y + 15, 20, BLACK);
    
    DrawRectangle(button_start_x + button_width + 10, button_start_y, button_width, button_height, RED);
    
    // Draw platform creation preview
    if (creating_platform && mouse_pressed) {
        Vector2 size = { 
            platform_current_pos.x - platform_start_pos.x,
            platform_current_pos.y - platform_start_pos.y
        };
        
        // Calculate center position for the platform
        Vector2 center = {
            platform_start_pos.x + size.x * 0.5f,
            platform_start_pos.y + size.y * 0.5f
        };
        
        // Draw preview rectangle
        DrawRectanglePro(
            (Rectangle){ center.x, center.y, fabs(size.x), fabs(size.y) }, 
            (Vector2){ fabs(size.x) * 0.5f, fabs(size.y) * 0.5f }, 
            0.0f, 
            Fade(BEIGE, 0.5f)
        );
        
        // Draw border
        DrawRectangleLinesEx(
            (Rectangle){ 
                platform_start_pos.x, 
                platform_start_pos.y, 
                size.x, 
                size.y 
            }, 
            2.0f, 
            YELLOW
        );
    }
}

void Toolbox::Update(float dt, std::vector<Platform>& platforms) {
    if (!edit_mode) return;
    
    // save platform configuration with 'J' key
    if (IsKeyPressed(KEY_J)) {
        SavePlatformConfiguration(platforms);
    }

    if (IsKeyPressed(KEY_L)){
        LoadPlatformConfiguration(platforms);
    }
    
    // toggle platform creation mode with '1' key
    if (IsKeyPressed(KEY_ONE)) {
        creating_platform = !creating_platform;
        mouse_pressed = false;
    }
    
    if (creating_platform) {
        Vector2 mouse_pos = GetMousePosition();
        
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && !mouse_pressed) {
            platform_start_pos = mouse_pos;
            mouse_pressed = true;
        }
        
        // update current position while dragging
        if (mouse_pressed) {
            platform_current_pos = mouse_pos;
        }
        
        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON) && mouse_pressed) {
            Vector2 size = { 
                platform_current_pos.x - platform_start_pos.x,
                platform_current_pos.y - platform_start_pos.y
            };
            
            // Only create platform if it has reasonable size
            if (fabs(size.x) > 10.0f && fabs(size.y) > 10.0f) {
                Vector2 center = {
                    platform_start_pos.x + size.x * 0.5f,
                    platform_start_pos.y + size.y * 0.5f
                };
                
                Platform new_platform(center, { fabs(size.x), fabs(size.y) }, 0.0f);
                platforms.push_back(new_platform);
            }
            
            mouse_pressed = false;
            creating_platform = false; // Exit creation mode after creating
        }
    }
}

// THIS WILL ONLY SAVE IN EDIT MODE
void Toolbox::SavePlatformConfiguration(const std::vector<Platform>& platforms) {
    std::ofstream file("platform_config.txt");
    
    if (!file.is_open()) {
        // Could add error handling here if needed
        return;
    }
    
    for (const Platform& platform : platforms) {
        float cos_rot = cosf(platform.rotation * DEG2RAD);
        float sin_rot = sinf(platform.rotation * DEG2RAD);
        
        // Local corner positions (relative to center)
        Vector2 corners[4] = {
            { -platform.size.x * 0.5f, -platform.size.y * 0.5f }, // top-left
            {  platform.size.x * 0.5f, -platform.size.y * 0.5f }, // top-right
            {  platform.size.x * 0.5f,  platform.size.y * 0.5f }, // bottom-right
            { -platform.size.x * 0.5f,  platform.size.y * 0.5f }  // bottom-left
        };
        
        // in reality, this should just be the corner position without rotation because
        // the parser will most likely want to rotate the platform after the rectangle is created
        Vector2 world_corners[4];
        for (int i = 0; i < 4; i++) {
            world_corners[i].x = corners[i].x + platform.position.x;
            world_corners[i].y = corners[i].y + platform.position.y;
        }
        
        // going into file in the format: R, x1y1,x2y2,x3y3,x4y4, rotation
        file << "R, ";
        for (int i = 0; i < 4; i++) {
            file << "{ " << world_corners[i].x << " , " << world_corners[i].y << " }";
            if (i < 3) file << ",";
        }
        file << ", " << platform.rotation << std::endl;
    }

    std::cout << "platform configuration saved to platform_config.txt" << std::endl;
    
    file.close();
}

void Toolbox::LoadPlatformConfiguration(std::vector<Platform>& platforms) {
    std::ifstream file("platform_config.txt");
    
    if (!file.is_open()) {
        std::cerr << "Failed to open platform_config.txt" << std::endl;
        return;
    }
    
    platforms.clear(); // clear existing
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        char type;
        float x1, y1, x2, y2, x3, y3, x4, y4, rotation;
        
        // Updated format string to handle spaces
        if (sscanf(line.c_str(), "%c, { %f , %f },{ %f , %f },{ %f , %f },{ %f , %f }, %f", 
                   &type, &x1, &y1, &x2, &y2, &x3, &y3, &x4, &y4, &rotation) == 10) {
            if (type == 'R') {
                Vector2 pos = { (x1 + x3) * 0.5f, (y1 + y3) * 0.5f }; // Center position
                Vector2 size = { fabs(x2 - x1), fabs(y3 - y1) }; // Size based on corners
                
                Platform new_platform(pos, size, rotation);
                platforms.push_back(new_platform);
            }
        }
    }
    
    std::cout << "platform configuration loaded from platform_config.txt" << std::endl;
    file.close();
}