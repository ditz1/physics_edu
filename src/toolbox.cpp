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
    std::string base_filename = "platform_config";
    std::string extension = ".txt";
    std::string filename = base_filename + extension;
    
    // Check if base filename exists, if so increment until we find an available name
    int counter = 1;
    std::ifstream test_file(filename);
    while (test_file.good()) {
        test_file.close();
        filename = base_filename + "_" + std::to_string(counter) + extension;
        test_file.open(filename);
        counter++;
    }
    test_file.close();
    
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        return;
    }
    
    for (const Platform& platform : platforms) {
        file << "R, " << platform.position.x << ", " << platform.position.y 
             << ", " << platform.size.x << ", " << platform.size.y 
             << ", " << platform.rotation << std::endl;
    }

    std::cout << "platform configuration saved to " << filename << std::endl;
    
    file.close();
}