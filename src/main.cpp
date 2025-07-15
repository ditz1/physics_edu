#include "../include/raylib/raylib.h"
#include "../include/toolbox.hpp"

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


int main(void) {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "physics engine");


    //DisableCursor();
    SetTargetFPS(60);
    Spring spring;
    spring.anchor = { screenWidth / 2.0f, screenHeight / 2.0f - 200.0f };
    spring.position = { screenWidth / 2.0f, screenHeight / 2.0f + 50.0f };
    // make it spawn to left so it falls down
    spring.mass = 2.0f;
    spring.spring_constant = 2.0f;
    spring.damping_factor = 0.05f;
    spring.rest_length = 250.0f;

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

    Vector2 plat_start = { screenWidth/2, screenHeight };
    Vector2 plat_size = { screenWidth, 320.0f };
    Platform platform = {plat_start, plat_size};

    Vector2 plat_center_2 = { 375.0f, screenHeight - 50.0f }; // center of platform
    Vector2 plat_size_2 = { 1050.0f, 325.0f };
    float plat_rotation = 40.0f; // degrees
    Platform platform2 = {plat_center_2, plat_size_2, plat_rotation};
    all_platforms.reserve(10);
    all_platforms.push_back(platform);
    all_platforms.push_back(platform2);

    while (!WindowShouldClose()) {
        
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
        box.CheckGrab();
        if (box.is_grabbed) {
            Vector2 mouse_position = GetMousePosition();
            box.Grab(mouse_position);
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

            // platform.CheckGrab();
            // if (platform.is_grabbed) {
            //     Vector2 mouse_position = GetMousePosition();
            //     platform.Grab(mouse_position);
            //     if (IsKeyDown(KEY_Z)){
            //         platform.rotation += 1.0f; // rotate platform clockwise
            //     } else if (IsKeyDown(KEY_C)) {
            //         platform.rotation -= 1.0f; // rotate platform counter-clockwise
            //     }
            // }
            // platform2.CheckGrab();
            // if (platform2.is_grabbed) {
            //     Vector2 mouse_position = GetMousePosition();
            //     platform2.Grab(mouse_position);
            //     if (IsKeyDown(KEY_Z)){
            //         platform2.rotation += 1.0f; // rotate platform clockwise
            //     } else if (IsKeyDown(KEY_C)) {
            //         platform2.rotation -= 1.0f; // rotate platform counter-clockwise
            //     }
            // }
        

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
            platform.Update(dt);
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
            

            ///////////////
            // SCENE 0_a ////
            ///////////////
            // spring.Draw();
            // // spring2.Draw();c
            // for (size_t i = 0; i < spring_path.size(); i++) {
            //     DrawCircleV(spring_path[i], 2.0f, GREEN);
            // }
            // if (toolbox_active) {
            //     toolbox.Draw();
            // }
            // DrawDebugInfo(spring);
            // spring.DrawVectors();

            // /////////////
            // SCENE 0_b ////
            // /////////////
            // spring2.Draw();
            // for (size_t i = 0; i < spring_path.size(); i++) {
            //     DrawCircleV(spring_path[i], 2.0f, GREEN);
            // }
            // DrawDebugInfo(spring);
            // DrawDebugInfo2(spring, spring2);
            // spring.Draw();
            // spring.DrawVectors();
            // spring2.Draw();
            // spring2.DrawVectors();
            // box.DrawVectors();

            /////////////////
            //// SCENE 1 ////
            /////////////////
            // spring.Draw();
            // spring.DrawVectors();
            // DrawDebugInfo(spring);
            // for (Star s : stars){
            //     s.Draw();
            // }
            // DrawText(TextFormat("Points: %d", points), 10, 10, 20, RAYWHITE);

            ///////////////
            // SCENE 2_a ////
            ///////////////
            // DrawText("get the box to the green square!", 400, 200, 20, RAYWHITE);
            // Rectangle box_rect = { (float)GetScreenWidth() - 200, (float)GetScreenHeight() - 100, 100, 100 };
            // DrawRectangleRec(box_rect, GREEN);
            // box.Draw();
            // box.DrawVectors();

            // if (CheckCollisionRecs(box_rect, box.Rect())){
            //     DrawText("Nice!", box_rect.x - 100, box_rect.y - 100, 20, GREEN);
            // }

            //  DrawText(TextFormat("Force: %.2f", box_force), 10, 10, 20, RAYWHITE);
            //  DrawText(TextFormat("Box Velocity: (%.2f, %.2f)", box.velocity.x, box.velocity.y), 10, 40, 20, RAYWHITE);
            //  DrawText(TextFormat("Box Friction (mu_f): %.2f", box.mu_kinetic), 10, 70, 20, RAYWHITE);
            // DrawFPS(10, 10);

            ///////////////
            // SCENE 2_b ////
            ///////////////

            

            DrawText("get the box to the green square!", 400, 200, 20, RAYWHITE);

            // Check collisions with both platforms
            // box.CheckPlatformCollisionSAT(platform, 1);  // Platform ID 1 (higher priority)
            // if (!box.is_colliding) {
            //     box.CheckPlatformCollisionSAT(platform2, 0);   // Platform ID 0 (lower priority)
            // }

            // Set color based on collision state
            if (box.is_colliding){
                box.color = PINK;
            } else {
                box.color = RED;
            }
            Rectangle box_rect = { screenWidth - 200, 480, 100, 100 };
            // platform.Draw();
            // platform2.Draw();
            for (Platform& plat : all_platforms) {
                plat.Draw();
            }
            DrawRectangleRec(box_rect, GREEN);
            box.Draw();
            box.DrawVectors();

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

    CloseWindow();

    return 0;
}