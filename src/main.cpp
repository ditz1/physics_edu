#include "../include/raylib/raylib.h"
#include "../include/toolbox.hpp"
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


    //DisableCursor();
    SetTargetFPS(60);


     bool config_loaded = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            // Load configuration from specified file
            LoadPlatformConfigurationFromFile(argv[i + 1], all_platforms);
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
        
        //all_platforms.push_back(platform);
        //all_platforms.push_back(platform2);
    }


    Spring spring;
    spring.anchor = { 500.0f, 500.0f };
    spring.position = { 500.0f, 750.0f };
    // make it spawn to left so it falls down
    spring.mass = 2.0f;
    spring.spring_constant = 5.0f;
    spring.damping_factor = 0.3f;
    spring.rest_length = 250.0f;
    Platform spring_platform = {spring.position, { 200.0f, 50.0f }, 0.0f };
    spring_platform.position = spring.position;

    float dt = 1.0f / 30.0f; // assume fixed time step, but if 60fps its will be this anyway

    std::vector<Vector2> spring_path;
    spring_path.push_back(spring.position);

    Spring spring2;
    spring2.anchor = spring.position;
    spring2.position = { screenWidth / 2.0f, screenHeight / 2.0f + 100.0f };
    spring2.mass = 2.0f;
    spring2.spring_constant = 5.0f;
    spring2.damping_factor = 0.05f;
    spring2.rest_length = 100.0f;

    Box box = Box(50, 50);
    box.position = { 75.0f, screenHeight / 2.0f - 200.0f };
    box.mass = 100.0f;
    box.texture = &bananas_tex; 


    Toolbox toolbox;
    bool toolbox_active = true;

    std::vector<Star> stars;
    for (int i = 0; i < 10; i++) {
        Star star;
        star.position = { (float)GetRandomValue(0, screenWidth), (float)GetRandomValue(0, screenHeight) };
        stars.push_back(star);
    }
    int points = 0;
    float box_force = 50.0f;

    BallAndString ball_and_string = BallAndString({ screenWidth / 2.0f, screenHeight / 2.0f }, 200.0f, 0.0f);
    ball_and_string.path.push_back(ball_and_string.position);
    
    all_platforms.push_back(spring_platform);
    Platform& spring_platform_ref = all_platforms.back();

  

    while (!WindowShouldClose()) {


        spring_platform_ref.position = spring.position;
        

        
        ball_and_string.Update(dt);
        if (Vector2Distance(ball_and_string.path.back(), ball_and_string.position) > 5.0f) {
            // only add to path if the position has changed significantly
            ball_and_string.path.push_back(ball_and_string.position);
        }

        

        spring.CheckGrab();
        if (spring.is_grabbed) {
            Vector2 mouse_position = GetMousePosition();
            spring.Grab(mouse_position);
        }


        bool was_grabbed_last_frame = box.is_grabbed;
        box.CheckGrab();
        if (box.is_grabbed) {
            Vector2 mouse_position = GetMousePosition();
            box.Grab(mouse_position);
            box.ghost_calculated = false; // reset ghost calculation when grabbed
        }

        // Check if box was just released
        if (was_grabbed_last_frame && !box.is_grabbed) {
            box.SetPredictionStartPosition();
        }

        spring2.CheckGrab();
        if (spring2.is_grabbed) {
            Vector2 mouse_position = GetMousePosition();
            spring2.Grab(mouse_position);
        }


        for (Platform& platform : all_platforms) {
            platform.CheckGrab();
            if (platform.is_grabbed) {
                platform.is_selected = true; // mark platform as selected
                Vector2 mouse_position = GetMousePosition();
                platform.Grab(mouse_position);
            } else {
                platform.is_selected = false; // mark platform as not selected
            }
            if (platform.is_selected) {
                if (IsKeyDown(KEY_Z)){
                    platform.rotation += 1.0f; // rotate platform clockwise
                } else if (IsKeyDown(KEY_C)) {
                    platform.rotation -= 1.0f; // rotate platform counter-clockwise
                }
            }
        }

        

        points = 0;
        for (Star& s : stars){
            s.CheckCollision(spring);
            if (s.is_grabbed) {
                points++;
            }
        }


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
            box.velocity = { box_force, 0.0f }; // reset velocity
        }

        if (IsKeyPressed(KEY_PERIOD)){
            box.mu_kinetic += 0.1f;
        } else if (IsKeyPressed(KEY_COMMA)){
            box.mu_kinetic -= 0.1f;
        }

        if (IsKeyPressed(KEY_E)){
            edit_mode = !edit_mode;
        }

        //////////////////
        // UPDATE LOGIC //
        //////////////////
        if (!edit_mode) {
            spring2.anchor = spring.position;   

            spring.Update(dt);
            spring2.Update(dt);

            box.Update(dt);
            box.was_colliding_last_frame = box.is_colliding; // store collision state for next frame
            box.last_platform_id = box.current_platform_id; // store last platform ID for next frame
            box.CheckCollision();
            //platform.Update(dt);
            spring_path.push_back(spring.position);

            ball_and_string.Update(dt);
        }

      

        if (toolbox_active) {
            toolbox.Update(dt, all_platforms);
        }

        for (auto& plat : all_platforms) {
            plat.Draw();
        }

        bool wasColliding = box.is_colliding;
        box.is_colliding = false;

        for (size_t i = 0; i < all_platforms.size(); i++) {
            if (!box.is_colliding) {
                box.CheckPlatformCollisionSAT(all_platforms[i], i);
            }
        }

        if (IsKeyDown(KEY_UP)) {
            ball_and_string.angularSpeed += 0.01f;
        } else if (IsKeyDown(KEY_DOWN)) {
            ball_and_string.angularSpeed -= 0.01f;
        }

        if (IsKeyPressed(KEY_SPACE)){
            ball_and_string.Break();
        }

        if (IsKeyPressed(KEY_R)) {
            ball_and_string.Reset();
        }

        if (ball_and_string.path.size() > 30){
            ball_and_string.path.erase(ball_and_string.path.begin());
        }

        BeginDrawing();
            ClearBackground(DARKGRAY);
            if (toolbox_active) {
                toolbox.Draw();
            }
            

            

            DrawText("get the box to the green square!", 400, 200, 20, RAYWHITE);

            // Check collisions with both platforms
            // box.CheckPlatformCollisionSAT(platform, 1);  // Platform ID 1 (higher priority)
            // if (!box.is_colliding) {
            //     box.CheckPlatformCollisionSAT(platform2, 0);   // Platform ID 0 (lower priority)
            // }

            // Set color based on collision state
            if (wasColliding){
                box.color = PINK;
            } else {
                box.color = RED;
            }
            
            if (wasColliding) {
                box.DrawMultiPlatformGhost(all_platforms);
            }

            Rectangle box_rect = { screenWidth - 200, 480, 100, 100 };
            // platform.Draw();
            // platform2.Draw();
            spring.Draw();
            for (Platform& plat : all_platforms) {
                plat.Draw();
            }

            DrawRectangleLinesEx(box_rect, 1.0f, GREEN);
            box.Draw();
            box.DrawVectors();
            DrawTextureEx(gorilla_tex, {box_rect.x, box_rect.y}, 0.0f, 0.125f, WHITE);

            if (CheckCollisionRecs(box_rect, box.Rect())){
                DrawText("Nice!", box_rect.x - 100, box_rect.y - 100, 20, GREEN);
            }

             DrawText(TextFormat("Force: %.2f", box_force), 10, 10, 20, RAYWHITE);
             DrawText(TextFormat("Box Velocity: (%.2f, %.2f)", box.velocity.x, box.velocity.y), 10, 40, 20, RAYWHITE);
             DrawText(TextFormat("Box Friction (mu_f): %.2f", box.mu_kinetic), 10, 70, 20, RAYWHITE);
            DrawFPS(10, 10);

            ///////////////
            // SCENE 3////
            ///////////////

            // ball_and_string.Draw();
            // ball_and_string.DrawVectors();
            // DrawText(TextFormat("Angular Speed: %.2f", ball_and_string.angularSpeed), 10, 10, 20, RAYWHITE);
            // for (size_t i = 0; i < ball_and_string.path.size(); i++) {
            //     DrawCircleV(ball_and_string.path[i], 2.0f, GREEN);
            // }

            if (edit_mode) {
                DrawText("EDIT MODE", screenWidth - 140, screenHeight - 40, 20, RED);
            }
            DrawText(TextFormat("Num Platforms: %d", all_platforms.size()), 50, 10, 20, RAYWHITE);
            if (toolbox.creating_platform) {
                DrawText("Creating Platform", 50, 40, 20, RAYWHITE);
            }
        EndDrawing();

        if (spring_path.size() > 100){
            spring_path.erase(spring_path.begin());
        }
    }

    UnloadTexture(gorilla_tex);
    UnloadTexture(bananas_tex);
    CloseWindow();

    return 0;
}