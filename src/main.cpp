#include "../include/raylib/raylib.h"
#include "../include/toolbox.hpp"
#include "../include/level_selector.hpp"
#include "../include/gorilla.hpp"
#include "../include/camera.hpp"
#include <cstring>
#include <string>
#include <cstdlib>

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

// ── User display name helper (Windows + POSIX) ──────────────────────────────────

#ifdef _WIN32
  #ifndef NOMINMAX
  #define NOMINMAX
  #endif
  #include <windows.h>
  static std::string GetSystemUserDisplayName() {
      WCHAR wbuf[256];
      DWORD n = (DWORD)(sizeof(wbuf)/sizeof(wbuf[0]));
      if (GetUserNameW(wbuf, &n)) {
          int need = WideCharToMultiByte(CP_UTF8, 0, wbuf, -1, nullptr, 0, nullptr, nullptr);
          if (need > 0) {
              std::string out(need - 1, '\0');
              WideCharToMultiByte(CP_UTF8, 0, wbuf, -1, out.data(), need, nullptr, nullptr);
              if (!out.empty()) return out;
          }
      }
      const char* u = std::getenv("USERNAME");
      if (u && *u) return std::string(u);
      return "Player";
  }
#else
  #include <pwd.h>
  #include <unistd.h>
  static std::string GetSystemUserDisplayName() {
      if (passwd* pw = getpwuid(geteuid())) {
          // Prefer full name (GECOS) up to the first comma
          if (pw->pw_gecos && pw->pw_gecos[0]) {
              std::string full = pw->pw_gecos;
              size_t c = full.find(',');
              if (c != std::string::npos) full.resize(c);
              // trim spaces
              auto trim = [](std::string& s){
                  size_t b = s.find_first_not_of(" \t");
                  size_t e = s.find_last_not_of(" \t");
                  if (b == std::string::npos) { s.clear(); return; }
                  s = s.substr(b, e - b + 1);
              };
              trim(full);
              if (!full.empty()) return full;
          }
          if (pw->pw_name && pw->pw_name[0]) return std::string(pw->pw_name);
      }
      const char* u = std::getenv("USER");
      if (u && *u) return std::string(u);
      return "Player";
  }
#endif


// Replace your old signature:
//   void LoadPlatformConfigurationFromFile(const char* filename, std::vector<Platform>& platforms)
//
// With this version that ALSO loads BOX and GORILLA positions, and assigns platform IDs.
bool LoadPlatformConfigurationFromFile(const char* filename,
    std::vector<Platform>& platforms,
    Box& box,
    Gorilla& gorilla) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return false;
    }

    platforms.clear();

    std::string line;
    int platform_count = 0;

    bool box_position_loaded = false;
    bool gorilla_position_loaded = false;

    // Reasonable defaults match your other loaders
    Vector2 default_box_position = { 75.0f, (float)GetScreenHeight() / 2.0f - 200.0f };
    Vector2 default_gorilla_position = { (float)GetScreenWidth() - 150.0f, 480.0f };

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        // BOX, x, y
        if (line.rfind("BOX", 0) == 0) {
            float bx, by;
            size_t first_comma = line.find(',');
            if (first_comma != std::string::npos) {
                std::string coords = line.substr(first_comma + 1);
                    if (sscanf(coords.c_str(), " %f, %f", &bx, &by) == 2) {
                        Vector2 box_pos = { bx, by };
                        box.position = box_pos;
                        box.origin_position = box_pos;
                        box_position_loaded = true;
            }
        }
        }
// GORILLA, x, y
        else if (line.rfind("GORILLA", 0) == 0) {
        float gx, gy;
        size_t first_comma = line.find(',');
            if (first_comma != std::string::npos) {
        std::string coords = line.substr(first_comma + 1);
        if (sscanf(coords.c_str(), " %f, %f", &gx, &gy) == 2) {
        gorilla.position = { gx, gy };
        gorilla_position_loaded = true;
        }
        }
        }
        // R, cx, cy, w, h, rot  (platform rows)
        else {
        char type;
        float cx, cy, w, h, rot;
        if (sscanf(line.c_str(), "%c, %f, %f, %f, %f, %f",
        &type, &cx, &cy, &w, &h, &rot) == 6) {
        if (type == 'R') {
        Vector2 pos  = { cx, cy };
        Vector2 size = { w, h };
        Platform p(pos, size, rot);
        p.id = platform_count;
        platforms.push_back(p);
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

        // Ensure platform IDs are consistent
        for (int i = 0; i < (int)platforms.size(); ++i) {
        platforms[i].id = i;
        }

        std::cout << "Platform configuration loaded from " << filename
        << " - " << platform_count << " platforms loaded" << std::endl;

        file.close();
        return true;
        }


int main(int argc, char* argv[]) {
    int screenWidth = 1280;
    int screenHeight = 720;

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "Banana Slide");

    // Initialize level selector and camera
    LevelSelector level_selector;
    SimCamera camera;

    // --- Audio setup ---
    InitAudioDevice();

    // Load tracks for tiers (looping)
    Music musicTier1 = LoadMusicStream("../assets/steeldrum.mp3");
    Music musicTier2 = LoadMusicStream("../assets/carribean.mp3");
    Music musicTier3 = LoadMusicStream("../assets/jungle.mp3");
    musicTier1.looping = true;
    musicTier2.looping = true;
    musicTier3.looping = true;

    // Optional: volume
    SetMusicVolume(musicTier1, 0.8f);
    SetMusicVolume(musicTier2, 0.3f);
    SetMusicVolume(musicTier3, 0.6f);

    // Current tier comes straight from the level selector: 1-1, 1-2, 1-3 => 1; 2-1.. => 2
    auto CurrentTier = [&]()->int {
        return level_selector.selected_level; // first number before the dash
    };

    Music* currentMusic = nullptr;
    int activeTier = 0;

    Texture2D gorilla_tex = LoadTexture("../assets/gorilla.png");
    Texture2D bananas_tex = LoadTexture("../assets/bananas.png");
    Texture2D log_tex = LoadTexture("../assets/log.png");
    Texture2D log_end_tex = LoadTexture("../assets/log_end.png");
    Texture2D log_slice_tex = LoadTexture("../assets/log_slice.png");
    Texture2D background_tex = LoadTexture("../assets/tropical_bg.png");
    Texture2D background_tex_2 = LoadTexture("../assets/sunset_bg.png");
    Texture2D background_tex_3 = LoadTexture("../assets/night_bg.png");

    auto SwitchToTier = [&](int tier) {
        Music* next = nullptr;
        if (tier == 1) next = &musicTier1;
        else if (tier == 2) next = &musicTier2;
        else if (tier == 3) next = &musicTier3;
        else {
            // no track defined for this tier — do nothing
            return;
        }

        if (currentMusic && IsMusicStreamPlaying(*currentMusic)) {
            StopMusicStream(*currentMusic);
        }
        currentMusic = next;
        PlayMusicStream(*currentMusic);
        activeTier = tier;
    };

    Texture2D* currentBackground = nullptr;

    auto SwitchBackground = [&](int tier) {
        Texture2D* next = nullptr;
        if (tier == 1) next = &background_tex;
        else if (tier == 2) next = &background_tex_2;
        else if (tier == 3) next = &background_tex_3;

        if (next) currentBackground = next;   // <— assign!
    };


    // Kick off the initial track for the currently selected level
    SwitchBackground(CurrentTier());
    SwitchToTier(CurrentTier());


    
    Color background_color = {0, 0, 0, 100};

    float dt_modifier = 1.0f;
    SetTargetFPS(60);

    int debug_font_size = 16;

    

    // Transition camera store
    static Camera2D stored_camera;
    static bool camera_stored = false;

    // Level switching variables
    bool collision_with_gorilla = false;
    float level_switch_timer = 0.0f;

    const float LEVEL_SWITCH_DELAY = 3.0f;
    bool showing_success = false;
    bool loading_next_level = false;
    float loading_timer = 0.0f;
    const float LOADING_DURATION = 1.0f;

    // --- App state for title / menus ---
    enum class AppState { TITLE, LEVEL_SELECT, PLAYING };
    AppState app_state = AppState::TITLE;
    bool request_exit = false;

    // Cached user name for the title screen
    static std::string gUserDisplayName = GetSystemUserDisplayName();



    // --- NEW: capture an optional startup config path, but DO NOT load yet ---
    const char* startup_config_path = nullptr;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            startup_config_path = argv[i + 1];
            break;
        }
    }

    float dt = 1.0f / 30.0f;

    // Create objects BEFORE loading, so we can assign positions from file
    Box box = Box(50, 50);
    box.position = { 75.0f, screenHeight / 2.0f - 200.0f };
    box.mass = 100.0f;
    box.texture = &bananas_tex;
    box.ghost_calculated = false;

    Gorilla gorilla({screenWidth - 150.0f, 480.0f});
    gorilla.texture = &gorilla_tex;

    // Now we can load config if provided so gorilla/box positions are applied
    bool config_loaded = false;
    if (startup_config_path) {
        config_loaded = LoadPlatformConfigurationFromFile(startup_config_path, all_platforms, box, gorilla);
        if (config_loaded) {
            for (int i = 0; i < (int)all_platforms.size(); i++) all_platforms[i].id = i;
            std::cout << "Loaded platform configuration from: " << startup_config_path << std::endl;
        }
    }

    // If nothing was loaded, fall back to your default platforms (unchanged)
    if (!config_loaded) {
        std::cout << "Using default platform configuration" << std::endl;
        Vector2 plat_start = { static_cast<float>(screenWidth/2.0f), static_cast<float>(screenHeight) };
        Vector2 plat_size = { static_cast<float>(screenWidth), 320.0f };
        Platform platform = {plat_start, plat_size};

        Vector2 plat_center_2 = { 375.0f, screenHeight - 50.0f };
        Vector2 plat_size_2 = { 1050.0f, 325.0f };
        float plat_rotation = 40.0f;
        Platform platform2 = {plat_center_2, plat_size_2, plat_rotation};

        all_platforms.push_back(platform);
        all_platforms.push_back(platform2);

        for (int i = 0; i < (int)all_platforms.size(); i++) {
            all_platforms[i].id = i;
        }
    }



   Toolbox toolbox;
   bool toolbox_active = false;
   float box_force = 50.0f;

   while (!WindowShouldClose()) {
        screenWidth = GetScreenWidth();
        screenHeight = GetScreenHeight();
       dt = 1.0f / (30.0f * dt_modifier);

                if (currentMusic) UpdateMusicStream(*currentMusic);

        // --- TITLE SCREEN ---
        if (app_state == AppState::TITLE) {
            BeginDrawing();
            // Background (use whichever tier is currently bound)
            if (currentBackground) {
                DrawTexture(*currentBackground, 0, 0, WHITE);
            } else {
                ClearBackground(BLACK);
            }
            // Dim overlay for readability
            DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), (Color){0, 0, 0, 150});

            // Title
            const char* title = "Banana Slide";
            int titleSize = 60;
            int titleW = MeasureText(title, titleSize);
            int titleX = GetScreenWidth()/2 - titleW/2;
            int titleY = GetScreenHeight()/2 - 220;
            DrawText(title, titleX, titleY, titleSize, RAYWHITE);

            // Welcome, {user}
            std::string welcome = std::string("Welcome, ") + gUserDisplayName;
            int welcomeSize = 28;
            int welcomeW = MeasureText(welcome.c_str(), welcomeSize);
            DrawText(welcome.c_str(),
         GetScreenWidth()/2 - welcomeW/2,
         GetScreenHeight() - 200,   // a little below the main title
         welcomeSize, RAYWHITE);


            // Simple button helper
            auto Button = [&](Rectangle r, const char* label) -> bool {
                Vector2 m = GetMousePosition();
                bool hover = CheckCollisionPointRec(m, r);
                Color fill = hover ? BROWN : BEIGE;
                DrawRectangleRounded(r, 0.2f, 8, fill);
                DrawRectangleRoundedLines(r, 0.2f, 8, WHITE);
                int fs = 28;
                int tw = MeasureText(label, fs);
                DrawText(label, (int)(r.x + (r.width - tw)/2), (int)(r.y + (r.height - fs)/2), fs, WHITE);
                return hover && IsMouseButtonPressed(MOUSE_LEFT_BUTTON);
            };

            // Buttons layout
            float bw = 300.0f, bh = 60.0f, gap = 20.0f;
            float cx = GetScreenWidth()/2.0f - bw/2.0f;
            float totalH = 3.0f*bh + 2.0f*gap;
            float y0 = GetScreenHeight()/2.0f - totalH/2.0f;
            Rectangle rStart  = { cx, y0,                 bw, bh };
            Rectangle rSelect = { cx, y0 + bh + gap,      bw, bh };
            Rectangle rQuit   = { cx, y0 + 2*(bh+gap),    bw, bh };

            // Button actions
            if (Button(rStart, "Start")) {
                level_selector.selected_level = 1;
                level_selector.selected_variant = 1;
                if (level_selector.LoadSelectedLevel(all_platforms, box, gorilla)) {
                    // Reset physics & level state (same as you do after ENTER in level selector)
                    box.velocity = { 0.0f, 0.0f };
                    box.acceleration = { 0.0f, 0.0f };
                    box.ghost_calculated = false;
                    box.multi_platform_ghost_calculated = false;
                    box.has_prediction_start = false;
                    box.is_colliding = false;

                    collision_with_gorilla = false;
                    showing_success = false;
                    loading_next_level = false;
                    level_switch_timer = 0.0f;
                    loading_timer = 0.0f;

                    SwitchBackground(CurrentTier());
                    SwitchToTier(CurrentTier());

                    app_state = AppState::PLAYING;
                }
            }

            if (Button(rSelect, "Level Select")) {
                level_selector.is_active = true;
                app_state = AppState::LEVEL_SELECT;
            }

            if (Button(rQuit, "Quit")) {
                request_exit = true;
            }

            EndDrawing();
            if (request_exit) break;
            continue; // skip the rest of the frame while on the title screen
        }

       // Handle level selector
       if (IsKeyPressed(KEY_M)) {
           level_selector.is_active = !level_selector.is_active;
       }



       if (IsKeyPressed(KEY_K)) {
            level_selector.SaveCurrentLevel(all_platforms, box, gorilla);
    
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
                SwitchBackground(CurrentTier());
                SwitchToTier(CurrentTier());

                // Enter gameplay after choosing a level
                app_state = AppState::PLAYING;

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
               dt_modifier += 0.51f;
           } else if (IsKeyPressed(KEY_EQUAL)){
               dt_modifier -= 0.51f;
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

       // Handle loading screen and transitions
       if (loading_next_level) {
           if (level_selector.is_transitioning) {
               // Update the sequential transition
               level_selector.UpdateSequentialTransition(GetFrameTime(), all_platforms, box, gorilla);

               // Reset physics state during transition
               box.velocity = { 0.0f, 0.0f };
               box.acceleration = { 0.0f, 0.0f };
               box.ghost_calculated = false;
               box.multi_platform_ghost_calculated = false;
               box.has_prediction_start = false;
               box.is_colliding = false;
           } else {
               loading_timer -= GetFrameTime();

               if (loading_timer <= 0.0f) {
                   // Start the sequential transition
                   level_selector.LoadNewLevel(all_platforms, box, gorilla);

                   if (!level_selector.is_transitioning) {
                       // If transition didn't start (error), finish loading
                       loading_next_level = false;
                       collision_with_gorilla = false;
                       showing_success = false;
                       level_switch_timer = 0.0f;
                       loading_timer = 0.0f;
                   }
               }
           }

           // Check if transition is complete
           if (!level_selector.is_transitioning && loading_timer <= 0.0f) {
               loading_next_level = false;
               collision_with_gorilla = false;
               showing_success = false;
               level_switch_timer = 0.0f;
               loading_timer = 0.0f;
               std::cout << "Level transition complete!" << std::endl;
           }
       }

       if (!level_selector.is_active) {
            int desiredTier = CurrentTier();
            if (desiredTier != activeTier) {
                SwitchBackground(desiredTier);
                SwitchToTier(desiredTier);
            }
        }
    

       BeginDrawing();
           ClearBackground(DARKGRAY);
           if (currentBackground) {
            Rectangle src = {0, 0, (float)currentBackground->width, (float)currentBackground->height};
            Rectangle dst = {0, 0, (float)screenWidth, (float)screenHeight};
            DrawTexturePro(*currentBackground, src, dst, {0,0}, 0, WHITE);
        }
            DrawRectangle(0, 0, screenWidth, screenHeight, background_color);    

           // Always show the game view (no more black loading screen)
           if (!level_selector.is_active) {
               // Camera handling during transitions
               if (level_selector.is_transitioning) {
                   // Use stored camera during transition to keep view stable
                   if (camera_stored) {
                       camera.camera = stored_camera;
                   }
               } else {
                   // Normal camera operation - update bounds
                   camera.FindBounds(all_platforms);
                   stored_camera = camera.camera; // Store for next transition
                   camera_stored = true;
               }
           
               BeginMode2D(camera.camera);
           
                   for (Platform& plat : all_platforms) {
                       plat.Draw();
                   }
               
                   if (box.has_prediction_start && !loading_next_level) {
                       box.DrawMultiPlatformGhost(all_platforms);
                   }
               
                   // Update and draw gorilla
                   gorilla.Update(dt);
                   gorilla.Draw();
               
                   box.Draw();

                   // Only draw vectors when not transitioning
                   if (!loading_next_level) {
                       box.DrawVectors();
                       box.DrawTwoLineCollisionDebug(all_platforms);
                   }
               
               EndMode2D();
               
               // TRANSITION DEBUG INFO (drawn outside camera view)
               if (level_selector.is_transitioning) {
                   DrawText("SEQUENTIAL TRANSITION IN PROGRESS", 10, 10, 20, RED);
                   
                   // Show current state
                   const char* state_names[] = {
                       "IDLE", "LOADING_NEW_PLATFORMS", "MOVING_OUT_OLD_PLATFORMS", 
                       "MOVING_IN_NEW_PLATFORMS", "MOVING_BOX_AND_GORILLA", "CLEANUP_COMPLETE"
                   };
                   const char* current_state_name = state_names[(int)level_selector.current_transition_state];
                   DrawText(TextFormat("State: %s", current_state_name), 10, 40, 16, YELLOW);
                   
                   // Show platform counts
                   DrawText(TextFormat("Old platforms: %d, New platforms: %d", 
                                      (int)level_selector.old_platforms_queue.size(),
                                      (int)level_selector.new_platforms_queue.size()), 10, 60, 14, WHITE);
                   
                   // Show platform positions
                   int y_offset = 90;
                   int platform_count = 0;
                   for (size_t i = 0; i < all_platforms.size() && platform_count < 6; i++, platform_count++) {
                       DrawText(TextFormat("Platform %d: (%.0f, %.0f)", platform_count, all_platforms[i].position.x, all_platforms[i].position.y), 
                                10, y_offset + platform_count * 20, 14, WHITE);
                   }
               }
               
               // Draw success message and countdown
               if (showing_success) {
                   DrawText("Nice!", gorilla.position.x - 50, gorilla.position.y - 100, 20, GREEN);
                   DrawText(TextFormat("Next level in: %.1f", level_switch_timer), gorilla.position.x - 80, gorilla.position.y - 70, 16, YELLOW);

                   // Draw success overlay
                   DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), (Color){0, 255, 0, 30});
               }
           
               // Draw transition overlay and progress
               if (loading_next_level) {
                   // Semi-transparent overlay during transition
                   DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), (Color){0, 0, 0, 50});

                   if (level_selector.is_transitioning) {
                       // Show current transition state
                       const char* state_names[] = {
                           "IDLE", "LOADING_NEW_PLATFORMS", "MOVING_OUT_OLD_PLATFORMS", 
                           "MOVING_IN_NEW_PLATFORMS", "MOVING_BOX_AND_GORILLA", "CLEANUP_COMPLETE"
                       };
                       
                       const char* current_state_name = state_names[(int)level_selector.current_transition_state];
                       const char* transition_text = TextFormat("Transition: %s", current_state_name);
                       int text_width = MeasureText(transition_text, 24);
                       DrawText(transition_text, (GetScreenWidth() - text_width) / 2, 50, 24, WHITE);

                       // Show platform movement progress
                       if (level_selector.current_transition_state == TransitionState::MOVING_OUT_OLD_PLATFORMS) {
                           DrawText(TextFormat("Moving out platform: %d / %d", 
                                              (int)level_selector.current_old_platform_index + 1, 
                                              (int)level_selector.old_platforms_queue.size()), 
                                    (GetScreenWidth() - 200) / 2, 80, 16, YELLOW);
                       } else if (level_selector.current_transition_state == TransitionState::MOVING_IN_NEW_PLATFORMS) {
                           DrawText(TextFormat("Moving in platform: %d / %d", 
                                              (int)level_selector.current_new_platform_index + 1, 
                                              (int)level_selector.new_platforms_queue.size()), 
                                    (GetScreenWidth() - 200) / 2, 80, 16, GREEN);
                       } else if (level_selector.current_transition_state == TransitionState::MOVING_BOX_AND_GORILLA) {
                           float progress = level_selector.entity_move_timer / level_selector.ENTITY_MOVE_DURATION;
                           DrawText(TextFormat("Moving box and gorilla: %.0f%%", progress * 100.0f), 
                                    (GetScreenWidth() - 200) / 2, 80, 16, BLUE); // Changed from CYAN to LIGHTBLUE
                       }
                   } else {
                       // Show initial loading message
                       float progress = 1.0f - (loading_timer / LOADING_DURATION);
                       const char* loading_text = "Preparing Level...";
                       int text_width = MeasureText(loading_text, 24);
                       DrawText(loading_text, (GetScreenWidth() - text_width) / 2, 50, 24, WHITE);

                       // Progress bar
                       int bar_width = 300;
                       int bar_height = 8;
                       int bar_x = (GetScreenWidth() - bar_width) / 2;
                       int bar_y = 80;

                       DrawRectangle(bar_x, bar_y, bar_width, bar_height, DARKGRAY);
                       DrawRectangle(bar_x, bar_y, (int)(bar_width * progress), bar_height, GREEN);
                       DrawRectangleLines(bar_x, bar_y, bar_width, bar_height, WHITE);
                   }
               }
           
               if (toolbox_active && !loading_next_level) {
                   toolbox.Draw(); 
               }
           
               if (!loading_next_level && !level_selector.is_transitioning) {
                   //DrawText("get the box to the green square!", 400, 200, 20, RAYWHITE);
               
                   // Debug ghost calculation status
                   if (box.has_prediction_start) {
                       DrawText(TextFormat("Multi-platform ghost: %s", box.multi_platform_ghost_calculated ? "YES" : "NO"), 10, 100, debug_font_size, RAYWHITE);
                       DrawText(TextFormat("Is colliding: %s", box.is_colliding ? "YES" : "NO"), 10, 130, debug_font_size, RAYWHITE);
                       DrawText(TextFormat("Trajectory segments: %d", (int)box.trajectory_segments.size()), 10, 160, debug_font_size, RAYWHITE);
                   }

                   // Show reference frame info
                   if (box.has_prediction_start && !box.trajectory_segments.empty()) {
                       int reference_frame_count = 0;
                       for (size_t i = 0; i < box.trajectory_segments.size(); i += 2) {
                           reference_frame_count++;
                       }
                       DrawText(TextFormat("Reference frames: %d", reference_frame_count), 10, 190, debug_font_size, RAYWHITE);
                   }
               
                   DrawText(TextFormat("Force: %.2f", box_force), 10, 10, debug_font_size, RAYWHITE);
                   DrawText(TextFormat("Box Velocity: (%.2f, %.2f)", box.velocity.x, box.velocity.y), 10, 40, debug_font_size, RAYWHITE);
                   DrawText(TextFormat("Box Friction (mu_f): %.2f", box.mu_kinetic), 10, 70, debug_font_size, RAYWHITE);
                   DrawFPS(10, 10);
               
                   if (edit_mode) {
                       DrawText("EDIT MODE", screenWidth - 140, screenHeight - 40, debug_font_size, RED);
                       DrawText("Grab platform + D = Delete", screenWidth - 200, screenHeight - 70, 16, YELLOW);
                   }

                   DrawText(TextFormat("Num Platforms: %d", all_platforms.size()), 50, 10, debug_font_size, RAYWHITE);
                   if (toolbox.creating_platform) {
                       DrawText("Creating Platform", 50, 40, 20, RAYWHITE);
                   }

                   // Show level selector instruction
                   DrawText("Press M for Level Select", 10, screenHeight - 30, 16, YELLOW);
               }
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
   UnloadTexture(background_tex);
   UnloadTexture(background_tex_2);
   UnloadTexture(background_tex_3);

   if (currentMusic && IsMusicStreamPlaying(*currentMusic)) StopMusicStream(*currentMusic);
    UnloadMusicStream(musicTier1);
    UnloadMusicStream(musicTier2);
    UnloadMusicStream(musicTier3);
    CloseAudioDevice();

   
   CloseWindow();

   return 0;
}