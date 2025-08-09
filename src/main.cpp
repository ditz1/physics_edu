#include "../include/raylib/raylib.h"
#include "../include/toolbox.hpp"
#include "../include/level_selector.hpp"
#include "../include/gorilla.hpp"
#include "../include/camera.hpp"
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

Vector2 GetWorldMousePosition(Camera2D camera) {
    return GetScreenToWorld2D(GetMousePosition(), camera);
}

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

    Texture2D gorilla_tex = LoadTexture("../assets/gorilla.png");
    Texture2D bananas_tex = LoadTexture("../assets/bananas.png");
    Texture2D log_tex = LoadTexture("../assets/log.png");
    Texture2D log_end_tex = LoadTexture("../assets/log_end.png");  // NEW
    Texture2D log_slice_tex = LoadTexture("../assets/log_slice.png");  // NEW

    float dt_modifier = 1.0f;

    SetTargetFPS(60);

    // Initialize level selector
    LevelSelector level_selector;
    SimCamera camera;

    // Level switching variables
    bool collision_with_gorilla = false;
    float level_switch_timer = 0.0f;
    const float LEVEL_SWITCH_DELAY = 3.0f; // 2 seconds to show success message
    bool showing_success = false;
    bool loading_next_level = false;
    float loading_timer = 0.0f;
    const float LOADING_DURATION = 1.0f; // 1 second loading screen

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
        Vector2 plat_start = { screenWidth/2.0f, screenHeight };
        Vector2 plat_size = { screenWidth, 320.0f };
        Platform platform = {plat_start, plat_size};

        Vector2 plat_center_2 = { 375.0f, screenHeight - 50.0f };
        Vector2 plat_size_2 = { 1050.0f, 325.0f };
        float plat_rotation = 40.0f;
        Platform platform2 = {plat_center_2, plat_size_2, plat_rotation};
        
        all_platforms.push_back(platform);
        all_platforms.push_back(platform2);
        
        // Assign IDs to default platforms
        for (int i = 0; i < all_platforms.size(); i++) {
            all_platforms[i].id = i;
        }
    }

    float dt = 1.0f / 30.0f; // Fixed time step for 60 FPS

    Box box = Box(50, 50);
    box.position = { 75.0f, screenHeight / 2.0f - 200.0f };
    box.mass = 100.0f;
    box.texture = &bananas_tex;
    box.ghost_calculated = false;

    // Initialize Gorilla
    Gorilla gorilla({screenWidth - 150.0f, 480.0f});
    gorilla.texture = &gorilla_tex;

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
            // Reset level switching state
            collision_with_gorilla = false;
            showing_success = false;
            loading_next_level = false;
            level_switch_timer = 0.0f;
            loading_timer = 0.0f;
            std::cout << "Scene reset - box returned to origin" << std::endl;
        }

        if (level_selector.is_active) {
            level_selector.Update();
            
            // Load selected level
            if (IsKeyPressed(KEY_ENTER)) {
                if (level_selector.LoadSelectedLevel(all_platforms, box, gorilla)) {
                    level_selector.is_active = false;
                    
                    // Reset physics state for both box and gorilla
                    box.velocity = { 0.0f, 0.0f };
                    box.acceleration = { 0.0f, 0.0f };
                    box.ghost_calculated = false;
                    box.multi_platform_ghost_calculated = false;
                    box.has_prediction_start = false;
                    box.is_colliding = false;
                    
                    // Reset level switching state
                    collision_with_gorilla = false;
                    showing_success = false;
                    loading_next_level = false;
                    level_switch_timer = 0.0f;
                    loading_timer = 0.0f;
                    
                    std::cout << "Level loaded successfully!" << std::endl;
                }
            }
        }

        // assign platform textures
        for (int i = 0; i < all_platforms.size(); i++) {
            all_platforms[i].log_texture = &log_tex;          // Start piece
            all_platforms[i].log_end_texture = &log_end_tex;  // End piece
            all_platforms[i].log_slice_texture = &log_slice_tex; // Middle slice
        }

        // Only update game logic when level selector is not active and not loading
        if (!level_selector.is_active && !loading_next_level) {
            Vector2 world_mouse_pos = GetWorldMousePosition(camera.camera);
    
            bool was_grabbed_last_frame = box.is_grabbed;
            box.CheckGrab(world_mouse_pos);  // Pass world coordinates
            if (box.is_grabbed) {
                box.Grab(world_mouse_pos);   // Pass world coordinates
                box.velocity = { 0.0f, 0.0f };
                box.acceleration = { 0.0f, 0.0f };
                box.ghost_calculated = false;
                
                // Reset level switching state when box is grabbed
                collision_with_gorilla = false;
                showing_success = false;
                level_switch_timer = 0.0f;
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
                
                platform.CheckResize(world_mouse_pos);  // Pass world coordinates
                if (platform.is_resizing) {
                    platform.HandleResize(world_mouse_pos);  // Pass world coordinates
                    platform.is_selected = true;
                } else {
                    platform.CheckGrab(world_mouse_pos);  // Pass world coordinates
                    if (platform.is_grabbed) {
                        platform.is_selected = true;
                        platform.Grab(world_mouse_pos);  // Pass world coordinates
                        
                        if (IsKeyPressed(KEY_D)) {
                            platform_to_delete = i;
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
            
            // Handle gorilla editing - ONLY in edit mode (same as platforms)
            if (edit_mode) {
                gorilla.CheckGrab(world_mouse_pos);  // Pass world coordinates
                if (gorilla.is_grabbed) {
                    gorilla.Grab(world_mouse_pos);   // Pass world coordinates
                }
            }
            
            // Delete the marked platform
            if (platform_to_delete >= 0) {
                all_platforms.erase(all_platforms.begin() + platform_to_delete);
                std::cout << "Deleted platform " << platform_to_delete << std::endl;
                
                // Reassign IDs to maintain consistency
                for (int i = 0; i < all_platforms.size(); i++) {
                    all_platforms[i].id = i;
                }
                
                // Reset box platform tracking to prevent invalid platform ID access
                box.current_platform_id = -1;
                box.last_platform_id = -1;
                box.is_colliding = false;
                box.was_colliding_last_frame = false;
                
                // Clear trajectory segments to prevent dangling platform pointers
                box.trajectory_segments.clear();
                box.projectile_trajectory_points.clear();
                
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
                toolbox.Update(dt, all_platforms, box, gorilla, world_mouse_pos);  // Pass world coordinates
            }

            // Check collision between box and gorilla
            if (gorilla.CheckCollisionWithBox(box) && !collision_with_gorilla) {
                collision_with_gorilla = true;
                showing_success = true;
                level_switch_timer = LEVEL_SWITCH_DELAY;
                std::cout << "Box reached gorilla! Level switch in " << LEVEL_SWITCH_DELAY << " seconds..." << std::endl;
            }

            // Handle level switching timer
            if (showing_success && level_switch_timer > 0.0f) {
                level_switch_timer -= dt;
                
                if (level_switch_timer <= 0.0f) {
                    // Start loading next level
                    showing_success = false;
                    loading_next_level = true;
                    loading_timer = LOADING_DURATION;
                    std::cout << "Starting level transition..." << std::endl;
                }
            }
        }

        // Handle loading screen
        if (loading_next_level) {
            loading_timer -= GetFrameTime();
            
            if (loading_timer <= 0.0f) {
                // Load next level
                level_selector.LoadNewLevel(all_platforms, box, gorilla);
                
                // Reset physics state for both box and gorilla
                box.velocity = { 0.0f, 0.0f };
                box.acceleration = { 0.0f, 0.0f };
                box.ghost_calculated = false;
                box.multi_platform_ghost_calculated = false;
                box.has_prediction_start = false;
                box.is_colliding = false;
                
                // Reset level switching state
                collision_with_gorilla = false;
                showing_success = false;
                loading_next_level = false;
                level_switch_timer = 0.0f;
                loading_timer = 0.0f;
                
                std::cout << "Next level loaded successfully!" << std::endl;
            }
        }

        BeginDrawing();
            ClearBackground(DARKGRAY);
            
            if (loading_next_level) {
                // Draw loading screen
                DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), (Color){0, 0, 0, 200});
                
                float loading_progress = 1.0f - (loading_timer / LOADING_DURATION);
                int bar_width = 400;
                int bar_height = 20;
                int bar_x = (GetScreenWidth() - bar_width) / 2;
                int bar_y = GetScreenHeight() / 2;
                
                // Draw loading bar background
                DrawRectangle(bar_x, bar_y, bar_width, bar_height, DARKGRAY);
                DrawRectangle(bar_x, bar_y, (int)(bar_width * loading_progress), bar_height, GREEN);
                DrawRectangleLines(bar_x, bar_y, bar_width, bar_height, WHITE);
                
                // Draw loading text
                const char* loading_text = "Loading Next Level...";
                int text_width = MeasureText(loading_text, 30);
                DrawText(loading_text, (GetScreenWidth() - text_width) / 2, bar_y - 50, 30, WHITE);
                
                // Draw percentage
                const char* percent_text = TextFormat("%.0f%%", loading_progress * 100);
                int percent_width = MeasureText(percent_text, 20);
                DrawText(percent_text, (GetScreenWidth() - percent_width) / 2, bar_y + 30, 20, WHITE);
            }
            else if (!level_selector.is_active) {
                camera.FindBounds(all_platforms);

                BeginMode2D(camera.camera);

                    for (Platform& plat : all_platforms) {
                        plat.Draw();
                    }

                    if (box.has_prediction_start) {
                        box.DrawMultiPlatformGhost(all_platforms);
                    }

                    // Update and draw gorilla
                    gorilla.Update(dt);
                    gorilla.Draw();

                    box.Draw();
                    box.DrawVectors();

                    box.DrawTwoLineCollisionDebug(all_platforms);

                EndMode2D();

                // Draw success message and countdown
                if (showing_success) {
                    DrawText("Nice!", gorilla.position.x - 50, gorilla.position.y - 100, 20, GREEN);
                    DrawText(TextFormat("Next level in: %.1f", level_switch_timer), gorilla.position.x - 80, gorilla.position.y - 70, 16, YELLOW);
                    
                    // Draw success overlay
                    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), (Color){0, 255, 0, 30});
                }

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
                
                // Show reference frame info
                if (box.has_prediction_start && !box.trajectory_segments.empty()) {
                    int reference_frame_count = 0;
                    for (size_t i = 0; i < box.trajectory_segments.size(); i += 2) {
                        reference_frame_count++;
                    }
                    DrawText(TextFormat("Reference frames: %d", reference_frame_count), 10, 190, 20, RAYWHITE);
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
    UnloadTexture(log_tex);
    UnloadTexture(log_end_tex);  
    UnloadTexture(log_slice_tex);
    
    CloseWindow();

    return 0;
}