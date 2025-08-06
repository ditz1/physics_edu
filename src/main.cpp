#include "../include/raylib/raylib.h"
#include "../include/toolbox.hpp"
#include "../include/level_selector.hpp"
#include <cstring>

bool edit_mode = false;
std::vector<Platform> all_platforms;



typedef class Star {
public:
    Vector2 position;
    float radius = 20.0f;
    Color color = YELLOW;
    bool is_grabbed = false;
    void Draw() {
        if (is_grabbed) return;
        DrawCircleV(position, radius, PURPLE);
        DrawCircleV(position, radius * 0.8f, YELLOW);
    }
    void CheckCollision(Spring spring){
        // check if spring position + radius (end of spring) collides with star
        if (CheckCollisionCircles(spring.position, spring.radius, position, radius)){
            is_grabbed = true;
        };
    }
} Star; 


void DrawDebugInfo(Spring spring){
    float x_start = GetScreenWidth() - 300.0f;
    float y_start = 10.0f;
    DrawText(TextFormat("F = -k * x"), x_start, y_start, 20, RAYWHITE);
    DrawText(TextFormat("(%.2f, %.2f) = -%.2f * %.2f", spring._force.x, spring._force.y, spring.spring_constant, spring._compression), x_start - 20, y_start + 20, 18, RAYWHITE);
    // pendulum
    // T = 2pi * sqrt(L / g)
    DrawText(TextFormat("T = 2pi * sqrt(L / g)"), x_start, y_start + 60, 20, RAYWHITE);
    float L = Vector2Distance(spring.anchor, spring.position);
    float g = 9.81f; // gravity
    float T = 2.0f * PI * sqrt(L / g);
    DrawText(TextFormat("%.2f = 2pi * sqrt(%.2f / %.2f)", T, L, g), x_start - 20, y_start + 80, 18, RAYWHITE);
}

void DrawDebugInfo2(Spring spring, Spring spring2){
    float x_start = GetScreenWidth() - 300.0f;
    float y_start = 50.0f;
    DrawText(TextFormat("T = 2pi * sqrt(L / g)"), x_start, y_start + 60, 20, RAYWHITE);
    float L_1 = Vector2Distance(spring.anchor, spring.position);
    float L_2 = Vector2Distance(spring2.anchor, spring2.position);
    float L = L_1 + L_2;
    float g = 9.81f; // gravity
    float T = 2.0f * PI * sqrt(L / g);
    DrawText(TextFormat("%.2f = 2pi * sqrt(%.2f / %.2f)", T, L, g), x_start - 20, y_start + 80, 18, RAYWHITE);
}

void LoadPlatformConfigurationFromFile(const char* filename, std::vector<Platform>& platforms) {
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }
    
    platforms.clear();
    
    std::string line;
    int platform_count = 0;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        char type;
        float center_x, center_y, width, height, rotation;
        
        if (sscanf(line.c_str(), "%c, %f, %f, %f, %f, %f", 
                   &type, &center_x, &center_y, &width, &height, &rotation) == 6) {
            if (type == 'R') {
                Vector2 pos = { center_x, center_y };
                Vector2 size = { width, height };
                
                Platform new_platform(pos, size, rotation);
                platforms.push_back(new_platform);
                platform_count++;
                std::cout << "Loaded platform " << platform_count << ": pos(" << pos.x << "," << pos.y 
                         << ") size(" << size.x << "," << size.y << ") rot(" << rotation << ")" << std::endl;
            }
        } else {
            std::cout << "Failed to parse line: " << line << std::endl;
        }
    }
    
    std::cout << "Platform configuration loaded from " << filename << " - " << platform_count << " platforms loaded" << std::endl;
    file.close();
}


int main(int argc, char* argv[]) {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "physics engine");

    Texture2D gorilla_tex = LoadTexture("../assets/gorilla1.png");
    Texture2D bananas_tex = LoadTexture("../assets/bananas.png");

    float dt_modifier = 1.0f;

    SetTargetFPS(60);

    // Initialize level selector
    LevelSelector level_selector;

    bool config_loaded = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            // Load configuration from specified file
            LoadPlatformConfigurationFromFile(argv[i + 1], all_platforms);
            for (int i = 0; i < all_platforms.size(); i++) {
                all_platforms[i].id = i;
            }
            config_loaded = true;
            std::cout << "Loaded platform configuration from: " << argv[i + 1] << std::endl;
            break;
        }
    }

    // Only add default platforms if no config was loaded
    if (!config_loaded) {
        std::cout << "Using default platform configuration" << std::endl;
        Vector2 plat_start = { screenWidth/2, screenHeight };
        Vector2 plat_size = { screenWidth, 320.0f };
        Platform platform = {plat_start, plat_size};

        Vector2 plat_center_2 = { 375.0f, screenHeight - 50.0f };
        Vector2 plat_size_2 = { 1050.0f, 325.0f };
        float plat_rotation = 40.0f;
        Platform platform2 = {plat_center_2, plat_size_2, plat_rotation};
        
        all_platforms.push_back(platform);
        all_platforms.push_back(platform2);
    }

    float dt = 1.0f / 30.0f; // Fixed time step for 60 FPS

    Box box = Box(50, 50);
    box.position = { 75.0f, screenHeight / 2.0f - 200.0f };
    box.mass = 100.0f;
    box.texture = &bananas_tex;
    box.ghost_calculated = false;

    Toolbox toolbox;
    bool toolbox_active = true;
    float box_force = 50.0f;

    while (!WindowShouldClose()) {
        dt = 1.0f / (30.0f * dt_modifier);

        // Handle level selector
        if (IsKeyPressed(KEY_M)) {
            level_selector.is_active = !level_selector.is_active;
        }

        if (IsKeyPressed(KEY_B)) {
            box.ResetToOrigin();
            std::cout << "Scene reset - box returned to origin" << std::endl;
        }

        if (level_selector.is_active) {
            level_selector.Update();
            
            // Load selected level
            if (IsKeyPressed(KEY_ENTER)) {
                if (level_selector.LoadSelectedLevel(all_platforms, box)) {
                    level_selector.is_active = false;
                    
                    // Box position is now loaded from file, just reset physics state
                    box.velocity = { 0.0f, 0.0f };
                    box.acceleration = { 0.0f, 0.0f };
                    box.ghost_calculated = false;
                    box.multi_platform_ghost_calculated = false;
                    box.has_prediction_start = false;
                    box.is_colliding = false;
                    
                    std::cout << "Level loaded successfully!" << std::endl;
                }
            }
        }

        // Only update game logic when level selector is not active
        if (!level_selector.is_active) {
            bool was_grabbed_last_frame = box.is_grabbed;
            box.CheckGrab();
            if (box.is_grabbed) {
                Vector2 mouse_position = GetMousePosition();
                box.Grab(mouse_position);
                box.velocity = { 0.0f, 0.0f };
                box.acceleration = { 0.0f, 0.0f };
                box.ghost_calculated = false; // reset ghost calculation when grabbed
            }

            // Check if box was just released
            if (was_grabbed_last_frame && !box.is_grabbed) {
                // Reset ghost calculation when box is released
                box.ghost_calculated = false;
                box.multi_platform_ghost_calculated = false;
                box.has_prediction_start = false;
                
                // If box is already on a platform when released, set prediction start immediately
                if (box.is_colliding) {
                    box.SetPredictionStartPosition();
                    box.CalculateGhostTrajectory(all_platforms);
                }
            }

            // Handle platform editing and deletion (only when not in level selector)
            int platform_to_delete = -1;
            
            for (int i = 0; i < all_platforms.size(); i++) {
                Platform& platform = all_platforms[i];
                
                platform.CheckResize();
                if (platform.is_resizing) {
                    Vector2 mouse_position = GetMousePosition();
                    platform.HandleResize(mouse_position);
                    platform.is_selected = true;
                } else {
                    platform.CheckGrab();
                    if (platform.is_grabbed) {
                        platform.is_selected = true;
                        Vector2 mouse_position = GetMousePosition();
                        platform.Grab(mouse_position);
                        
                        if (IsKeyPressed(KEY_D)) {
                            platform_to_delete = i;
                            std::cout << "Marking platform " << i << " for deletion" << std::endl;
                        }
                    } else {
                        platform.is_selected = false;
                    }
                }
                
                if (platform.is_selected) {
                    if (IsKeyDown(KEY_Z)){
                        platform.rotation += 1.0f;
                    } else if (IsKeyDown(KEY_C)) {
                        platform.rotation -= 1.0f;
                    }
                }
            }
            
            // Delete the marked platform
            if (platform_to_delete >= 0) {
                all_platforms.erase(all_platforms.begin() + platform_to_delete);
                std::cout << "Deleted platform " << platform_to_delete << std::endl;
                
                box.ghost_calculated = false;
                box.multi_platform_ghost_calculated = false;
                box.has_prediction_start = false;
                
                if (box.is_colliding) {
                    box.SetPredictionStartPosition();
                    box.CalculateGhostTrajectory(all_platforms);
                }
            }

            // Handle other key inputs (only when level selector is not active)
            if (IsKeyPressed(KEY_TAB)){
                toolbox_active = !toolbox_active;
            }

            if (IsKeyPressed(KEY_LEFT)){
                box_force -= 1.0f;
            } else if (IsKeyPressed(KEY_RIGHT)){
                box_force += 1.0f;
            }
            if (IsKeyPressed(KEY_SPACE)){
                box.acceleration = { box_force, 0.0f };
                box.velocity = { box_force, 0.0f };
            }

            if (IsKeyPressed(KEY_PERIOD)){
                box.mu_kinetic += 0.01f;
            } else if (IsKeyPressed(KEY_COMMA)){
                box.mu_kinetic -= 0.01f;
            }

            if (IsKeyPressed(KEY_SEMICOLON)){
                box.mu_kinetic -= 0.1f;
            } else if (IsKeyPressed(KEY_APOSTROPHE)){
                box.mu_kinetic += 0.1f;
            }

            if (IsKeyPressed(KEY_MINUS)){
                dt_modifier += 0.5f;
            } else if (IsKeyPressed(KEY_EQUAL)){
                dt_modifier -= 0.5f;
            }

            if (IsKeyPressed(KEY_E)){
                edit_mode = !edit_mode;
            }

            // Update game logic
            if (!edit_mode) {
                box.was_colliding_last_frame = box.is_colliding;
                box.last_platform_id = box.current_platform_id;
                
                box.CheckPlatformCollisionTwoLine(all_platforms);
                box.Update(dt, all_platforms);
            }

            if (toolbox_active) {
                toolbox.Update(dt, all_platforms, box);
            }
        }

        for (auto& plat : all_platforms) {
            plat.Draw();
        }

        BeginDrawing();
            ClearBackground(DARKGRAY);
            
            // Only draw game UI when level selector is not active
            if (!level_selector.is_active) {
                if (toolbox_active) {
                    toolbox.Draw(); 
                }

                DrawText("get the box to the green square!", 400, 200, 20, RAYWHITE);

                // Debug ghost calculation status
                if (box.has_prediction_start) {
                    DrawText(TextFormat("Multi-platform ghost: %s", box.multi_platform_ghost_calculated ? "YES" : "NO"), 10, 100, 20, RAYWHITE);
                    DrawText(TextFormat("Is colliding: %s", box.is_colliding ? "YES" : "NO"), 10, 130, 20, RAYWHITE);
                    DrawText(TextFormat("Trajectory segments: %d", (int)box.trajectory_segments.size()), 10, 160, 20, RAYWHITE);
                }
                
                // Draw multi-platform ghost trajectory using reference frames
                if (box.has_prediction_start) {
                    box.DrawMultiPlatformGhost(all_platforms);
                }
                
                // Show reference frame info
                if (box.has_prediction_start && !box.trajectory_segments.empty()) {
                    int reference_frame_count = 0;
                    for (size_t i = 0; i < box.trajectory_segments.size(); i += 2) {
                        reference_frame_count++;
                    }
                    DrawText(TextFormat("Reference frames: %d", reference_frame_count), 10, 190, 20, RAYWHITE);
                }

                Rectangle box_rect = { screenWidth - 200, 480, 100, 100 };
                
                for (Platform& plat : all_platforms) {
                    plat.Draw();
                }

                DrawRectangleLinesEx(box_rect, 1.0f, GREEN);
                box.Draw();
                box.DrawVectors();
                
                box.DrawTwoLineCollisionDebug(all_platforms);
                DrawTextureEx(gorilla_tex, {box_rect.x, box_rect.y}, 0.0f, 0.125f, WHITE);

                if (CheckCollisionRecs(box_rect, box.Rect())){
                    DrawText("Nice!", box_rect.x - 100, box_rect.y - 100, 20, GREEN);
                }

                DrawText(TextFormat("Force: %.2f", box_force), 10, 10, 20, RAYWHITE);
                DrawText(TextFormat("Box Velocity: (%.2f, %.2f)", box.velocity.x, box.velocity.y), 10, 40, 20, RAYWHITE);
                DrawText(TextFormat("Box Friction (mu_f): %.2f", box.mu_kinetic), 10, 70, 20, RAYWHITE);
                DrawFPS(10, 10);

                if (edit_mode) {
                    DrawText("EDIT MODE", screenWidth - 140, screenHeight - 40, 20, RED);
                    DrawText("Grab platform + D = Delete", screenWidth - 200, screenHeight - 70, 16, YELLOW);
                }
                
                DrawText(TextFormat("Num Platforms: %d", all_platforms.size()), 50, 10, 20, RAYWHITE);
                if (toolbox.creating_platform) {
                    DrawText("Creating Platform", 50, 40, 20, RAYWHITE);
                }
                
                // Show level selector instruction
                DrawText("Press M for Level Select", 10, screenHeight - 30, 16, YELLOW);
            }
            
            // Draw level selector (this will draw on top of everything)
            level_selector.Draw();
            
        EndDrawing();
    }

    UnloadTexture(gorilla_tex);
    UnloadTexture(bananas_tex);
    CloseWindow();

    return 0;
}