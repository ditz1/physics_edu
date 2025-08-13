#include "../include/box.hpp"
#include "../include/collision_utils.hpp"
#include <iostream>
#include <algorithm>
#include <limits>
#include <vector>
#include <set>

Box::Box() {
    size = {50.0f, 50.0f};
}
Box::Box(int w, int h) {
    size = {static_cast<float>(w), static_cast<float>(h)};
}

Box::~Box() {

}

// ===== Dimension/label helpers (pure UI) =====
static inline float vecLength(Vector2 v) { return sqrtf(v.x*v.x + v.y*v.y); }
static inline Vector2 vecSub(Vector2 a, Vector2 b) { return {a.x-b.x, a.y-b.y}; }
static inline Vector2 vecAdd(Vector2 a, Vector2 b) { return {a.x+b.x, a.y+b.y}; }
static inline Vector2 vecScale(Vector2 v, float s) { return {v.x*s, v.y*s}; }
static inline Vector2 vecNorm(Vector2 v) { float L=vecLength(v); return (L>1e-6f)?Vector2{v.x/L,v.y/L}:Vector2{0,0}; }
static inline Vector2 vecPerp(Vector2 v) { return {-v.y, v.x}; }


// === Analytic arc length for projectile: x(t)=v0x t, y(t)=v0y t + 0.5 g t^2 ===
// s(T) = ∫0→T sqrt( v0x^2 + (v0y + g t)^2 ) dt
static float ArcLenProjectile(Vector2 v0, float g, float T) {
    if (T <= 0.0f) return 0.0f;
    // Work in double for stability
    const double a = (double)v0.x * (double)v0.x + (double)v0.y * (double)v0.y; // v0^2
    const double b = 2.0 * (double)v0.y * (double)g;
    const double c = (double)g * (double)g; // g^2
    auto S = [&](double t){ return sqrt(c*t*t + b*t + a); };
    auto F = [&](double t){
        // Indefinite integral of sqrt(c t^2 + b t + a)
        const double ct = 2.0*c*t + b;
        const double St = S(t);
        const double k  = 4.0*a*c - b*b;       // >= 0 for typical projectile
        const double c32 = pow(c, 1.5);
        double term1 = (ct * St) / (4.0*c);
        // Guard log() arg
        double logarg = 2.0*sqrt(c)*St + ct;
        if (logarg < 1e-12) logarg = 1e-12;
        double term2 = (k / (8.0*c32)) * log(logarg);
        return term1 + term2;
    };
    double result = F((double)T) - F(0.0);
    if (!std::isfinite(result) || result < 0.0) {
        // Very rare: fall back to quick numeric quad (good enough for UI + totals)
        const int N = 64;
        double s = 0.0, dt = T / N;
        double vy = (double)v0.y;
        double vx = (double)v0.x;
        for (int i=0;i<N;i++){
            double t0 = i*dt, t1=(i+1)*dt;
            double v0m = hypot(vx, vy + g*t0);
            double v1m = hypot(vx, vy + g*t1);
            s += 0.5*(v0m+v1m)*dt;
        }
        result = s;
    }
    return (float)result;
}


// Compact text with a soft background for readability
static void DrawTextBg(const char* txt, Vector2 pos, int fontSize, Color fg, Color bg) {
    int w = MeasureText(txt, fontSize);
    int h = fontSize + 4;
    DrawRectangle((int)(pos.x - 4), (int)(pos.y - 2), w + 8, h, bg);
    DrawText(txt, (int)pos.x, (int)pos.y, fontSize, fg);
}

// Label a segment with its length, offset a bit off the segment
static void DrawSegmentLengthLabel(Vector2 A, Vector2 B, float overrideLen = -1.0f) {
    float d = (overrideLen >= 0.0f) ? overrideLen : Vector2Distance(A, B);
    Vector2 mid = { (A.x + B.x)*0.5f, (A.y + B.y)*0.5f };
    Vector2 n = vecPerp(vecNorm(vecSub(B, A)));   // normal
    Vector2 labelPos = vecAdd(mid, vecScale(n, -14.0f)); // slight offset above
    DrawTextBg(TextFormat("%.1f", d), labelPos, 14, BLACK, (Color){255,255,255,180});
}

// Draw horizontal & vertical gap dimensions between launch and land points
static void DrawGapDimensions(Vector2 launch, Vector2 land) {
    // Horizontal at launch height; vertical at land x
    Vector2 H0 = { launch.x, launch.y };
    Vector2 H1 = { land.x,   launch.y };
    Vector2 V0 = H1;
    Vector2 V1 = { land.x, land.y };

    // Lines
    DrawLineEx(H0, H1, 2.0f, SKYBLUE); // width
    DrawLineEx(V0, V1, 2.0f, LIME);    // height

    // End caps (small ticks)
    Vector2 hx = vecNorm(vecSub(H1, H0));
    Vector2 hy = vecPerp(hx);
    Vector2 vx = vecNorm(vecSub(V1, V0));
    Vector2 vy = vecPerp(vx);
    float tick = 6.0f;
    DrawLineEx(vecAdd(H0, vecScale(hy, -tick)), vecAdd(H0, vecScale(hy,  tick)), 2.0f, SKYBLUE);
    DrawLineEx(vecAdd(H1, vecScale(hy, -tick)), vecAdd(H1, vecScale(hy,  tick)), 2.0f, SKYBLUE);
    DrawLineEx(vecAdd(V0, vecScale(vy, -tick)), vecAdd(V0, vecScale(vy,  tick)), 2.0f, LIME);
    DrawLineEx(vecAdd(V1, vecScale(vy, -tick)), vecAdd(V1, vecScale(vy,  tick)), 2.0f, LIME);

    // Labels
    float dx = fabsf(land.x - launch.x);
    float dy = fabsf(land.y - launch.y);
    Vector2 hxLabelPos = { (H0.x + H1.x)*0.5f, H0.y - 16.0f };
    Vector2 vyLabelPos = { V1.x + 6.0f, (V0.y + V1.y)*0.5f - 7.0f };

    DrawTextBg(TextFormat("W=%.1f", dx), hxLabelPos, 14, BLACK, (Color){200,230,255,220});
    DrawTextBg(TextFormat("H=%.1f", dy), vyLabelPos, 14, BLACK, (Color){200,255,200,220});
}


void Box::Update(float dt, const std::vector<Platform>& platforms) {
    // Store the previous collision state
    was_colliding_last_frame = is_colliding;

    if (!is_colliding) {
        // Simple gravity when not colliding - acceleration = 9.81 m/s²
        velocity.y += gravity * dt;
    } else {
        // SIMPLE KINEMATICS when colliding with platform
        float slope_angle = rotation * DEG2RAD;
        
        // Get the slope direction vector
        Vector2 slope_direction = { cos(slope_angle), sin(slope_angle) };
        
        // Project current velocity onto the slope to get speed along slope
        float current_speed_along_slope = Vector2DotProduct(velocity, slope_direction);
        
        // Apply constant acceleration along slope: a = g * sin(θ)
        float slope_acceleration = gravity * sin(slope_angle);
        float new_speed_along_slope = current_speed_along_slope + slope_acceleration * dt;
        
        // Set velocity to be exactly along the slope direction with the new speed
        velocity = Vector2Scale(slope_direction, new_speed_along_slope);
    }
    
    // Apply simple friction
    ApplyFriction(dt);

    // Ensure left-to-right movement as per the rules
    if (is_colliding && velocity.x < 0) {
        velocity.x = 0; // Stop leftward movement
    }

    // REMOVE the "stuck" push logic - it causes twitching when box should be at rest
    // Box should naturally come to rest due to friction, not artificial pushes

    position = Vector2Add(position, Vector2Scale(velocity, dt));
}

void Box::Draw() {
    const float border = 0.2f;

    // --- Outer border (rotates around center) ---
    DrawRectanglePro(
        (Rectangle){ position.x, position.y, size.x, size.y },
        (Vector2){ size.x * 0.5f, size.y * 0.5f },
        rotation,
        BLACK
    );

    // --- Inner fill under the texture (optional but looks nicer if texture has alpha) ---
    DrawRectanglePro(
        (Rectangle){ position.x, position.y, size.x - border * 2.0f, size.y - border * 2.0f },
        (Vector2){ (size.x - border * 2.0f) * 0.5f, (size.y - border * 2.0f) * 0.5f },
        rotation,
        color
    );

    // --- Texture: preserve aspect, upscale, keep bottom aligned with cube ---
    if (texture && texture->width > 0 && texture->height > 0) {
        // Base target height = inner box height; scale more if you want it bigger
        const float innerH      = size.y + (size.y * 0.2f);
        const float heightScale = 1.5f;          // <== make it bigger/smaller here
        const float desiredH    = innerH * heightScale;

        // Keep aspect ratio: scale by height
        const float scale = desiredH / (float)texture->height;
        const float dstW  = (float)texture->width  * scale;
        const float dstH  = (float)texture->height * scale; // == desiredH

        // Compute cube bottom-center in world space (so bottom stays lined up when rotating)
        const float rad  = rotation * DEG2RAD;
        const float cs   = cosf(rad), sn = sinf(rad);

        // Align to OUTER bottom of the cube. If you prefer inner bottom, use: (size.y*0.5f - border)
        const float bottomOffsetY = size.y * 0.8f;

        Vector2 bottomCenter = {
            position.x + (0.0f * cs - bottomOffsetY * sn),
            position.y + (0.0f * sn + bottomOffsetY * cs)
        };

        // Draw with pivot at texture bottom-center
        Rectangle src = { 0.0f, 0.0f, (float)texture->width, (float)texture->height };
        Rectangle dst = { bottomCenter.x, bottomCenter.y, dstW, dstH };
        Vector2   origin = { dstW * 0.5f, dstH }; // bottom-center pivot

        DrawTexturePro(*texture, src, dst, origin, rotation, WHITE);
    }

    // Optional outline
    Rectangle outline = {
        position.x - size.x * 0.5f,
        position.y - size.y * 0.5f,
        size.x, size.y
    };
    DrawRectangleLinesEx(outline, border, BLACK);

    // Debug center marker
    // DrawCircleV(position, 5, WHITE);
}



void Box::CheckCollision() {
    int s_width = GetScreenWidth();
    int s_height = GetScreenHeight();

    if (position.x < 0.0f || position.x + (size.x * 0.5f) > s_width) {
        velocity.x *= -0.2f; // reverse x velocity
        position.x = (position.x < 0.0f) ? 0.0f : s_width - size.x; // reset position to screen bounds
    } 
    
    if (position.y < 0.0f || position.y + (size.y * 0.5f) > s_height) {
        velocity.y *= -0.2f; // reverse y velocity
        position.y = (position.y < 0.0f) ? 0.0f : s_height - size.y; // reset position to screen bounds
    } 

}

void Box::CheckPlatformCollision(Rectangle platform_rect) {
    // Check if the box is colliding with the platform
    if (CheckCollisionRecs(Rect(), platform_rect)) {
        // If the box is below the platform, reset its position to the top of the platform
        is_colliding = true;
        if (position.y + size.y > platform_rect.y && position.y < platform_rect.y + platform_rect.height) {
            position.y = platform_rect.y - (size.y * 0.5); // place box on top of the platform
            //velocity.y = 0.0f; // reset vertical velocity
        }
    }
}

// REMOVED: CheckPlatformCollisionSAT - was causing duplicate collision handling and velocity conflicts
// Now using only CheckPlatformCollisionTwoLine for consistent physics

void Box::CheckPlatformCollisionTwoLine(const std::vector<Platform>& platforms) {
    // FIRST: Check if we're at a transition point and need to teleport
    if (is_colliding && CheckAndHandleTransition(platforms)) {
        return; // Transition handled, collision will be rechecked next frame
    }
    
    // SIMPLE COLLISION: Check if box bottom is near any platform
    Vector2 boxBottom = { position.x, position.y + size.y * 0.5f };
    
    float closest_distance = std::numeric_limits<float>::max();
    int closest_platform_id = -1;
    Vector2 closest_point;
    
    // Find the closest platform, with priority for platforms in direction of movement
    for (size_t i = 0; i < platforms.size(); i++) {
        const Platform& platform = platforms[i];
        
        // Get platform direction and length
        Vector2 platformDir = Vector2Subtract(platform.top_right, platform.top_left);
        Vector2 toBox = Vector2Subtract(boxBottom, platform.top_left);
        float platformLength = Vector2Length(platformDir);
        
        if (platformLength > 0) {
            Vector2 normalizedDir = Vector2Normalize(platformDir);
            float t = Vector2DotProduct(toBox, normalizedDir) / platformLength;
            t = fmax(0.0f, fmin(1.0f, t)); // Clamp to platform bounds
            
            // Calculate the closest point on the platform surface
            Vector2 closestPointOnPlatform = Vector2Add(platform.top_left, Vector2Scale(normalizedDir, t * platformLength));
            
            // Calculate distance from box bottom to platform surface
            float distance = Vector2Distance(boxBottom, closestPointOnPlatform);
            
            // Check if box is above the platform and close enough to magnetize
            if (boxBottom.y >= closestPointOnPlatform.y - 5.0f && distance < 10.0f) {
                // PRIORITIZATION FIX: If box is moving rightward, prefer platforms to the right
                bool is_preferred = true;
                if (velocity.x > 0.1f && is_colliding && current_platform_id >= 0 && current_platform_id < (int)platforms.size()) {
                    // Check if this platform is in the direction of movement
                    Vector2 platform_center = Vector2Add(platform.top_left, Vector2Scale(platformDir, 0.5f));
                    if (platform_center.x < position.x - 10.0f) {
                        // This platform is behind the box - deprioritize it
                        is_preferred = false;
                    }
                }
                
                float effective_distance = is_preferred ? distance : distance + 5.0f; // Add penalty for non-preferred
                
                if (effective_distance < closest_distance) {
                    closest_distance = effective_distance;
                    closest_platform_id = (int)i;  // Cast to int for consistency
                    closest_point = closestPointOnPlatform;
                }
            }
        }
    }
    
    // FIXED: Add proper bounds checking before accessing platforms array
    if (closest_platform_id >= 0 && closest_platform_id < (int)platforms.size()) {
        // Collision detected
        bool isNewCollision = !was_colliding_last_frame;
        bool isPlatformTransition = (was_colliding_last_frame && last_platform_id != closest_platform_id);
        
        // Store old velocity for platform transitions
        float old_speed = Vector2Length(velocity);
        
        is_colliding = true;
        current_platform_id = closest_platform_id;
        
        // Get the platform we're colliding with
        const Platform& platform = platforms[closest_platform_id];
        rotation = platform.rotation;
        
        // MAGNETIZE the box to be exactly on the platform surface
        float distanceToMove = boxBottom.y - closest_point.y;
        position.y -= distanceToMove; // Always snap to surface, regardless of direction
        
        // Handle velocity
        if (isPlatformTransition) {
            // Simple transition: just follow the new platform direction
            float new_slope_angle = platform.rotation * DEG2RAD;
            Vector2 new_slope_direction = {cos(new_slope_angle), sin(new_slope_angle)};
            velocity = Vector2Scale(new_slope_direction, old_speed);
        } else if (isNewCollision) {
            // For new collisions, redirect velocity along platform
            float slope_angle = platform.rotation * DEG2RAD;
            Vector2 slope_direction = {cos(slope_angle), sin(slope_angle)};
            float speed_along_slope = Vector2DotProduct(velocity, slope_direction);
            velocity = Vector2Scale(slope_direction, fabs(speed_along_slope));
            
            // Set prediction start position when box first lands on a platform
            if (!has_prediction_start) {
                SetPredictionStartPosition();
            }
        }
    } else {
        // No collision
        if (was_colliding_last_frame) {
            is_colliding = false;
        }
    }
}

bool Box::CheckAndHandleTransition(const std::vector<Platform>& platforms) {
    if (!is_colliding || current_platform_id < 0 || current_platform_id >= platforms.size()) {
        return false;
    }
    
    const Platform& current_platform = platforms[current_platform_id];
    
    // Calculate the RIGHT BOTTOM CORNER of the box
    float box_rotation_rad = rotation * DEG2RAD;
    float cos_rot = cosf(box_rotation_rad);
    float sin_rot = sinf(box_rotation_rad);
    
    // Local coordinates of right bottom corner
    Vector2 local_right_bottom = { size.x * 0.5f, size.y * 0.5f };
    
    // Rotate and translate to world coordinates
    Vector2 box_right_corner = {
        local_right_bottom.x * cos_rot - local_right_bottom.y * sin_rot + position.x,
        local_right_bottom.x * sin_rot + local_right_bottom.y * cos_rot + position.y
    };
    
    // Check if the right corner is near the transition point
    Vector2 current_right_end = GetRightEndOfPlatform(current_platform);
    float distance_to_right_end = Vector2Distance(box_right_corner, current_right_end);
    
    // Threshold for transition detection
    float transition_threshold = 20.0f;
    
    if (distance_to_right_end < transition_threshold && velocity.x > 0) {
        // IMPROVED PROJECTILE DETECTION: Check if there's a next platform the box could reach
        const Platform* next_platform = FindNextPlatformToRight(current_platform, platforms);
        
        if (next_platform) {
            // Check if platforms are connected (intersection or very close)
            Vector2 intersection;
            bool platforms_connected = CheckCollisionLines(
                current_platform.top_left, current_platform.top_right,
                next_platform->top_left, next_platform->top_right, &intersection
            );
            
            // Also check proximity between platform endpoints
            Vector2 current_right_end_pos = GetRightEndOfPlatform(current_platform);
            Vector2 next_left_end = (next_platform->top_left.x < next_platform->top_right.x) ? 
                                   next_platform->top_left : next_platform->top_right;
            float gap_distance = Vector2Distance(current_right_end_pos, next_left_end);
            
            if (platforms_connected || gap_distance < 10.0f) {
                // Platforms are connected - use teleport transition
                Vector2 transition_point = platforms_connected ? intersection : current_right_end_pos;
                
                // Calculate where box should be positioned on the next platform
                Vector2 new_position = transition_point;
                new_position.y -= size.y * 0.5f; // Place box on platform surface
                
                // Store current speed for velocity preservation
                float current_speed = Vector2Length(velocity);
                
                // TELEPORT: Set new position and platform
                position = new_position;
                last_platform_id = current_platform_id;
                current_platform_id = -1; // Will be set by collision detection
                
                // Calculate new velocity direction along next platform
                float next_slope_angle = next_platform->rotation * DEG2RAD;
                Vector2 next_slope_direction = { cos(next_slope_angle), sin(next_slope_angle) };
                
                // Preserve speed, redirect along new platform
                velocity = Vector2Scale(next_slope_direction, current_speed);
                rotation = next_platform->rotation;
                
                return true; // Transition handled
            } else {
                // PROJECTILE MOTION: Platforms are not connected, box needs to jump/fall
                // Calculate launch velocity from current platform
                float slope_angle = current_platform.rotation * DEG2RAD;
                Vector2 slope_direction = { cos(slope_angle), sin(slope_angle) };
                float current_speed = Vector2Length(velocity);
                Vector2 launch_velocity = Vector2Scale(slope_direction, current_speed);
                
                // Box launches from end of current platform
                is_colliding = false;
                current_platform_id = -1;
                velocity = launch_velocity; // Set projectile velocity
                
                std::cout << "Box launching from platform " << current_platform.id << " to platform " << next_platform->id 
                          << " with velocity: (" << launch_velocity.x << ", " << launch_velocity.y << ")" << std::endl;
                return true; // Transition handled by launching
            }
        } else {
            // No next platform found - box falls off the end
            is_colliding = false;
            current_platform_id = -1;
            
            std::cout << "Box falling off end of platform " << current_platform.id << " - no next platform found" << std::endl;
            return true; // Let box continue in projectile motion
        }
    }
    
    return false; // No transition needed
}

void Box::ApplyFriction(float dt) {
    if (is_colliding) {
        float slope_angle = rotation * DEG2RAD;
        
        // CONSISTENT FRICTION with ghost prediction: μ * g * cos(θ)
        float normal_force_magnitude = gravity * cos(fabs(slope_angle));
        float friction_magnitude = mu_kinetic * normal_force_magnitude;
        
        // Get current speed along slope
        Vector2 slope_direction = {cos(slope_angle), sin(slope_angle)};
        float speed_along_slope = Vector2DotProduct(velocity, slope_direction);
        
        // Apply friction deceleration opposing motion
        if (fabs(speed_along_slope) > 0.001f) {
            float friction_deceleration = friction_magnitude;
            if (speed_along_slope < 0) friction_deceleration *= -1;
            
            float new_speed = speed_along_slope - friction_deceleration * dt;
            
            // Prevent friction from reversing velocity direction
            if ((speed_along_slope > 0 && new_speed < 0) || (speed_along_slope < 0 && new_speed > 0)) {
                // Stop completely if friction would reverse direction
                velocity = {0.0f, 0.0f};
            } else {
                // Apply the new speed along slope direction
                velocity = Vector2Scale(slope_direction, new_speed);
            }
        } else {
            // Already at rest - keep it at rest
            velocity = {0.0f, 0.0f};
        }
    }
}


float Box::CalculateStoppingDistanceFromSlope(const Platform& slope_platform, float slope_travel_distance) {
    // PURE KINEMATICS - no energy conservation, only friction slows down the box
    float slope_angle = slope_platform.rotation * DEG2RAD;
    
    // On slope: acceleration = g*sin(θ) - μ*g*cos(θ) (gravity minus friction)
    float gravity_component = gravity * sin(slope_angle);
    float friction_component = mu_kinetic * gravity * cos(fabs(slope_angle));
    float net_acceleration = gravity_component - friction_component;
    
    // Calculate final velocity after traveling down slope using: v² = u² + 2as
    // Starting from rest: v² = 2as
    float final_velocity_squared = 2.0f * net_acceleration * slope_travel_distance;
    
    if (final_velocity_squared <= 0) {
        return 0.0f; // Box stops on the slope
    }
    
    float final_velocity = sqrt(final_velocity_squared);
    
    // On horizontal surface: only friction decelerates
    // a = -μg, v² = u² + 2as, final v = 0
    // 0 = u² - 2μg*s → s = u²/(2μg)
    float horizontal_deceleration = mu_kinetic * gravity;
    float stopping_distance = (final_velocity * final_velocity) / (2.0f * horizontal_deceleration);
    
    return stopping_distance;
}

float Box::GetSlopeDistance(const Platform& platform) {
    // Calculate the actual distance along the slope surface
    Vector2 platform_direction = Vector2Subtract(platform.top_right, platform.top_left);
    return Vector2Length(platform_direction);
}

float Box::GetDistanceAlongSlope(Vector2 start_point, Vector2 end_point, const Platform& slope_platform) {
    // Project the start and end points onto the slope surface
    Vector2 slope_direction = Vector2Normalize(Vector2Subtract(slope_platform.top_right, slope_platform.top_left));
    
    // Calculate the position along the slope for both points
    Vector2 to_start = Vector2Subtract(start_point, slope_platform.top_left);
    Vector2 to_end = Vector2Subtract(end_point, slope_platform.top_left);
    
    float start_t = Vector2DotProduct(to_start, slope_direction);
    float end_t = Vector2DotProduct(to_end, slope_direction);
    
    // Return the distance along the slope
    return abs(end_t - start_t);
}

Vector2 Box::CalculateStoppingPosition(const Platform& slope_platform, const Platform& horizontal_platform) {
    float slope_distance = GetSlopeDistance(slope_platform);
    float horizontal_stopping_distance = CalculateStoppingDistanceFromSlope(slope_platform, slope_distance);
    
    // Calculate position on horizontal platform where box will stop
    Vector2 horizontal_direction = {1.0f, 0.0f}; // Assuming horizontal platform is flat
    Vector2 stopping_position = Vector2Add(horizontal_platform.position, 
                                          Vector2Scale(horizontal_direction, horizontal_stopping_distance));
    
    return stopping_position;
}

void Box::SetPredictionStartPosition() {
    // Only set prediction start if we haven't already set it
    if (has_prediction_start) {
        return;
    }
    
    // Calculate the BOTTOM RIGHT CORNER of the rotated box
    float box_angle = rotation * DEG2RAD;
    float cos_rot = cosf(box_angle);
    float sin_rot = sinf(box_angle);
    
    // Local coordinates of bottom right corner (relative to box center)
    Vector2 local_bottom_right = { size.x * 0.5f, size.y * 0.5f };
    
    // Rotate and translate to world coordinates
    prediction_start_position = {
        local_bottom_right.x * cos_rot - local_bottom_right.y * sin_rot + position.x,
        local_bottom_right.x * sin_rot + local_bottom_right.y * cos_rot + position.y
    };

    // move prediction point up slightly
    
    has_prediction_start = true;
    multi_platform_ghost_calculated = false;
    ghost_calculated = false; // Reset ghost calculation
    std::cout << "Prediction start set at bottom right corner: (" << prediction_start_position.x << ", " << prediction_start_position.y << ")" << std::endl;
    std::cout << "Box position: (" << position.x << ", " << position.y << "), rotation: " << rotation << std::endl;
}

void Box::CalculateGhostTrajectory(const std::vector<Platform>& platforms) {
    if (!has_prediction_start) {
        std::cout << "CalculateGhostTrajectory: No prediction start position!" << std::endl;
        return;
    }
    
    // Find the current platform (slope) and the next platform (horizontal)
    const Platform* slope_platform = nullptr;
    const Platform* horizontal_platform = nullptr;
    
    // Use the current platform that the box is colliding with
    if (current_platform_id >= 0 && current_platform_id < platforms.size()) {
        slope_platform = &platforms[current_platform_id];
        std::cout << "Using current collision platform " << current_platform_id << " with rotation " << slope_platform->rotation << std::endl;
    } else {
        // Fallback: search for platform near prediction start position
        for (size_t i = 0; i < platforms.size(); i++) {
            const auto& platform = platforms[i];
            bool is_on_platform = IsPointOnPlatform(prediction_start_position, platform);
            std::cout << "Platform " << i << " (rotation: " << platform.rotation << "): distance check = " << is_on_platform << std::endl;
            
            if (is_on_platform) {
                slope_platform = &platform;
                std::cout << "Found slope platform " << i << " with rotation " << platform.rotation << std::endl;
                break;
            }
        }
    }
    
    if (!slope_platform) {
        std::cout << "No slope platform found for ghost calculation! Current platform ID: " << current_platform_id 
                  << ", Prediction start: (" << prediction_start_position.x << ", " << prediction_start_position.y << ")" << std::endl;
        return;
    }
    
    // Find the horizontal platform that connects to this slope
    horizontal_platform = FindNextPlatformToRight(*slope_platform, platforms);
    
    if (!horizontal_platform) {
        std::cout << "No horizontal platform found for ghost calculation!" << std::endl;
        return;
    }
    
    // Find the intersection point between slope and horizontal platforms
    Vector2 intersect;
    bool found_intersection = CheckCollisionLines(slope_platform->top_left, slope_platform->top_right, 
                           horizontal_platform->top_left, horizontal_platform->top_right, &intersect);
    
    if (found_intersection) {
        transition_point_stored = intersect;
        
        // Calculate the distance along the slope surface from prediction start to transition point
        float slope_distance = GetDistanceAlongSlope(prediction_start_position, transition_point_stored, *slope_platform);
        
        // Calculate final velocity after traveling down slope, starting with current velocity
        float current_speed = Vector2Length(velocity);
        float final_velocity = CalculateFinalVelocity(current_speed, *slope_platform, slope_distance);
        
        // Calculate stopping distance on horizontal platform using this final velocity
        float horizontal_deceleration = mu_kinetic * gravity;
        float horizontal_stopping_distance = (final_velocity * final_velocity) / (2.0f * horizontal_deceleration);
        
        // Calculate the direction along the horizontal platform
        Vector2 horizontal_direction = Vector2Normalize(Vector2Subtract(horizontal_platform->top_right, horizontal_platform->top_left));
        
        // Calculate final ghost position
        ghost_position_stored = Vector2Add(transition_point_stored, 
                                         Vector2Scale(horizontal_direction, horizontal_stopping_distance));
        
        ghost_calculated = true;
        
        std::cout << "Ghost calculated - slope: " << slope_distance 
                  << ", horizontal: " << horizontal_stopping_distance 
                  << ", final: (" << ghost_position_stored.x << ", " << ghost_position_stored.y << ")" << std::endl;
        
        // Validate the calculation
        if (horizontal_stopping_distance < 0 || horizontal_stopping_distance > 10000) {
            std::cout << "WARNING: Invalid horizontal stopping distance: " << horizontal_stopping_distance << std::endl;
        }
    } else {
        // If no direct intersection, find the closest connection point for left-to-right movement
        Vector2 current_right = GetRightEndOfPlatform(*slope_platform);
        Vector2 next_left = (horizontal_platform->top_left.x < horizontal_platform->top_right.x) ? 
                           horizontal_platform->top_left : horizontal_platform->top_right;
        
        // For left-to-right movement, use the right end of slope as transition point
        transition_point_stored = current_right;
        
        // Calculate the distance along the slope surface from prediction start to transition point
        float slope_distance = GetDistanceAlongSlope(prediction_start_position, transition_point_stored, *slope_platform);
        
        // Calculate final velocity after traveling down slope, starting with current velocity
        float current_speed = Vector2Length(velocity);
        float final_velocity = CalculateFinalVelocity(current_speed, *slope_platform, slope_distance);
        
        // Calculate stopping distance on horizontal platform using this final velocity
        float horizontal_deceleration = mu_kinetic * gravity;
        float horizontal_stopping_distance = (final_velocity * final_velocity) / (2.0f * horizontal_deceleration);
        
        // Calculate the direction along the horizontal platform
        Vector2 horizontal_direction = Vector2Normalize(Vector2Subtract(horizontal_platform->top_right, horizontal_platform->top_left));
        
        // Calculate final ghost position
        ghost_position_stored = Vector2Add(transition_point_stored, 
                                         Vector2Scale(horizontal_direction, horizontal_stopping_distance));
        
        ghost_calculated = true;
        
        std::cout << "Ghost calculated (no intersection) - slope: " << slope_distance 
                  << ", horizontal: " << horizontal_stopping_distance 
                  << ", final: (" << ghost_position_stored.x << ", " << ghost_position_stored.y << ")" << std::endl;
    }
}

void Box::DrawGhost(const Platform& slope_platform, const Platform& horizontal_platform) {
    if (!has_prediction_start || !ghost_calculated) {
        std::cout << "DrawGhost: Not drawing - has_prediction_start: " << has_prediction_start 
                  << ", ghost_calculated: " << ghost_calculated << std::endl;
        return; // No prediction start position set yet or ghost not calculated
    }
    
    // std::cout << "DrawGhost: Drawing ghost at (" << ghost_position_stored.x << ", " << ghost_position_stored.y << ")" << std::endl;
    
    // Draw debug lines
    DrawLineEx(slope_platform.top_left, slope_platform.top_right, 2.0f, RED);
    DrawLineEx(horizontal_platform.top_left, horizontal_platform.top_right, 2.0f, BLUE);
    DrawCircleV(transition_point_stored, 5, GREEN);
    
    // Draw the predicted path from FIXED starting position
    DrawLineEx(prediction_start_position, transition_point_stored, 3.0f, ORANGE); // Down the slope
    DrawLineEx(transition_point_stored, ghost_position_stored, 3.0f, YELLOW); // Along horizontal
    
    // Draw starting position marker
    DrawCircleV(prediction_start_position, 4, PURPLE);
    
    // Draw ghost box at calculated position
    float border = 2.0f;
    DrawRectanglePro(
        (Rectangle){ ghost_position_stored.x, ghost_position_stored.y, size.x, size.y }, 
        (Vector2){ size.x * 0.5f, size.y * 0.5f }, 
        0.0f,
        (Color){255, 255, 255, 100}
    );
    
    DrawRectanglePro(
        (Rectangle){ ghost_position_stored.x, ghost_position_stored.y, size.x - border * 2, size.y - border * 2 }, 
        (Vector2){ (size.x - border * 2) * 0.5f, (size.y - border * 2) * 0.5f }, 
        0.0f,
        (Color){color.r, color.g, color.b, 80}
    );
    
    // Show debug info
    Vector2 text_pos = {ghost_position_stored.x - 50, ghost_position_stored.y - 40};
    float slope_distance = GetSlopeDistance(slope_platform);
    DrawText(TextFormat("Slope dist: %.1f", slope_distance), text_pos.x, text_pos.y, 14, ORANGE);
}

void Box::DrawMultiPlatformGhost(const std::vector<Platform>& platforms) {
    if (!has_prediction_start) {
        return;
    }
    
    if (!multi_platform_ghost_calculated) {
        CalculateMultiReferenceFrameTrajectory(platforms);
        multi_platform_ghost_calculated = true;
    }
    
    // Draw reference frame boxes (every 2 platforms = 1 frame)
    // std::vector<const Platform*> sorted_platforms;
    // for (const auto& platform : platforms) {
    //     sorted_platforms.push_back(&platform);
    // }
    
    // std::sort(sorted_platforms.begin(), sorted_platforms.end(), 
    //     [](const Platform* a, const Platform* b) {
    //         float a_left = std::min(a->top_left.x, a->top_right.x);
    //         float b_left = std::min(b->top_left.x, b->top_right.x);
    //         return a_left < b_left;
    //     });

        
    
    // // Draw reference frames
    // Color frame_colors[] = { PURPLE, MAGENTA, BLUE, DARKGREEN, ORANGE, PINK };
    // int num_colors = sizeof(frame_colors) / sizeof(frame_colors[0]);
    
    // for (int frame = 0; frame * 2 + 1 < (int)sorted_platforms.size(); frame++) {
    //     const Platform* slope_platform = sorted_platforms[frame * 2];
    //     const Platform* horizontal_platform = sorted_platforms[frame * 2 + 1];
        
    //     Color frame_color = frame_colors[frame % num_colors];
        
    //     // Calculate bounding box for this reference frame
    //     float min_x = std::min({slope_platform->top_left.x, slope_platform->top_right.x, 
    //     horizontal_platform->top_left.x, horizontal_platform->top_right.x}) - 20;
    //     float max_x = std::max({slope_platform->top_left.x, slope_platform->top_right.x,
    //                            horizontal_platform->top_left.x, horizontal_platform->top_right.x}) + 20;
    //     float min_y = std::min({slope_platform->top_left.y, slope_platform->top_right.y,
    //                            horizontal_platform->top_left.y, horizontal_platform->top_right.y}) - 50;
    //     float max_y = std::max({slope_platform->top_left.y, slope_platform->top_right.y,
    //                            horizontal_platform->top_left.y, horizontal_platform->top_right.y}) + 50;
        
    //     // Draw reference frame boundary
    //     DrawRectangleLines((int)min_x, (int)min_y, (int)(max_x - min_x), (int)(max_y - min_y), frame_color);
    //     DrawText(TextFormat("Frame %d", frame + 1), (int)min_x, (int)min_y - 20, 16, frame_color);
    // }
    // Draw reference frames — ONE per platform (left->right)
    // std::vector<const Platform*> sorted_platforms;
    // sorted_platforms.reserve(platforms.size());
    // for (const auto& p : platforms) sorted_platforms.push_back(&p);
    // std::sort(sorted_platforms.begin(), sorted_platforms.end(),
    //     [](const Platform* a, const Platform* b) {
    //         float ax = std::min(a->top_left.x, a->top_right.x);
    //         float bx = std::min(b->top_left.x, b->top_right.x);
    //         if (ax == bx) return a->id < b->id;
    //         return ax < bx;
    //     });

    // Color frame_colors[] = { PURPLE, MAGENTA, BLUE, DARKGREEN, ORANGE, PINK };
    // int num_colors = (int)(sizeof(frame_colors) / sizeof(frame_colors[0]));

    // for (size_t i = 0; i < sorted_platforms.size(); ++i) {
    //     const Platform* p = sorted_platforms[i];
    //     Color c = frame_colors[i % num_colors];

    //     // Simple visual: line along the top surface for each platform/frame
    //     DrawLineEx(p->top_left, p->top_right, 3.0f, c);

    //     // (Optional) tiny label
    //     // DrawText(TextFormat("F%zu", i+1), (int)((p->top_left.x + p->top_right.x)/2), (int)(std::min(p->top_left.y, p->top_right.y) - 14), 12, c);
    // }

    
    // Draw all trajectory segments (rest of the function stays exactly the same)
    // Draw all trajectory segments + length labels / gap dimensions
    for (size_t i = 0; i < trajectory_segments.size(); i++) {
        const auto& segment = trajectory_segments[i];

        if (segment.platform == nullptr) {
            // --- Projectile arc (keep your existing curved/polyline draw) ---
            if (!projectile_trajectory_points.empty()) {
                for (size_t j = 1; j < projectile_trajectory_points.size(); j++) {
                    DrawLineEx(projectile_trajectory_points[j-1], projectile_trajectory_points[j], 3.0f, MAGENTA);
                }
                for (const auto& pt : projectile_trajectory_points) {
                    DrawCircleV(pt, 2.0f, PINK);
                }
            }

            // --- Gap dimensions: horizontal width and vertical height from launch to land ---
            Vector2 launch = segment.start_position;
            Vector2 land   = segment.end_position;
            DrawGapDimensions(launch, land);

            // Optional: show straight-line chord length of the jump (for reference)
            DrawSegmentLengthLabel(launch, land);

        } else {
            // --- On-platform segment: draw and label its travel length ---
            Color line_color = (fabsf(segment.platform->rotation) > 5.0f) ? ORANGE : YELLOW;
            DrawLineEx(segment.start_position, segment.end_position, 5.0f, line_color);

            // Use stored distance if available (it includes along-slope measurement)
            float segLen = (segment.distance > 0.0f) 
                ? segment.distance 
                : Vector2Distance(segment.start_position, segment.end_position);

            DrawSegmentLengthLabel(segment.start_position, segment.end_position, segLen);
        }

        // Mark transitions/endpoints
        DrawCircleV(segment.end_position, 5, GREEN);
    }

    
    // Draw starting position
    DrawCircleV(prediction_start_position, 4, PURPLE);
    
    // Draw final ghost box
    if (!trajectory_segments.empty()) {
        float border = 2.0f;
        DrawRectanglePro(
            (Rectangle){ final_ghost_position.x, final_ghost_position.y, size.x, size.y }, 
            (Vector2){ size.x * 0.5f, size.y * 0.5f }, 
            0.0f,
            (Color){255, 255, 255, 100}
        );
                               
       DrawRectanglePro(
           (Rectangle){ final_ghost_position.x, final_ghost_position.y, size.x - border * 2, size.y - border * 2 }, 
           (Vector2){ (size.x - border * 2) * 0.5f, (size.y - border * 2) * 0.5f }, 
           0.0f,
           (Color){color.r, color.g, color.b, 80}
       );
   }
   
   // Draw debug info
   if (!trajectory_segments.empty()) {
       Vector2 text_pos = {final_ghost_position.x - 50, final_ghost_position.y - 60};
       float total_distance = 0;
       for (const auto& segment : trajectory_segments) {
           total_distance += segment.distance;
       }
       DrawText(TextFormat("Total dist: %.1f", total_distance), text_pos.x, text_pos.y, 14, WHITE);
       DrawText(TextFormat("Segments: %d", trajectory_segments.size()), text_pos.x, text_pos.y + 20, 14, WHITE);
       
       // Show how many reference frames are being calculated
       int num_frames = (trajectory_segments.size() + 1) / 2; // Rough estimate
       DrawText(TextFormat("Frames: %d", num_frames), text_pos.x, text_pos.y + 40, 14, WHITE);
   }
}

void Box::ResetToOrigin() {
   position = origin_position;
   velocity = {0.0f, 0.0f};
   acceleration = {0.0f, 0.0f};
   is_colliding = false;
   current_platform_id = -1;
   last_platform_id = -1;
   
   // Reset ghost calculations
   ghost_calculated = false;
   multi_platform_ghost_calculated = false;
   has_prediction_start = false;
   
   std::cout << "Box reset to origin position: (" << origin_position.x << ", " << origin_position.y << ")" << std::endl;
}

void Box::CalculateMultiPlatformTrajectory(const std::vector<Platform>& platforms) {
   trajectory_segments.clear();
   
   Vector2 current_position = prediction_start_position;
   float current_velocity = 0.0f; // Starting from rest
   const Platform* current_platform = nullptr;
   
   // Find the starting platform
   for (const auto& platform : platforms) {
       if (IsPointOnPlatform(current_position, platform)) {
           current_platform = &platform;
           break;
       }
   }
   
   if (!current_platform) {
       std::cout << "No starting platform found!" << std::endl;
       return;
   }
   
   int max_segments = 10;
   int segment_count = 0;
   
   while (current_platform && segment_count < max_segments) {
       TrajectorySegment segment;
       segment.platform = current_platform;
       segment.start_position = current_position;
       
       // Always move towards the RIGHT end of the current platform
       Vector2 platform_right_end = GetRightEndOfPlatform(*current_platform);
       float distance_to_right_end = Vector2Distance(current_position, platform_right_end);
       
       // Find the next platform that connects to the right
       const Platform* next_platform = FindNextPlatformToRight(*current_platform, platforms);
       Vector2 transition_point;
       bool has_transition = false;
       float distance_to_transition = distance_to_right_end;
       
       if (next_platform) {
           // Find intersection point between current and next platform
           if (CheckCollisionLines(current_platform->top_left, current_platform->top_right,
                                  next_platform->top_left, next_platform->top_right, &transition_point)) {
               // Make sure transition point is to the right of current position
               if (transition_point.x > current_position.x) {
                   has_transition = true;
                   distance_to_transition = Vector2Distance(current_position, transition_point);
               }
           }
       }
       
       Vector2 end_position;
       float distance_traveled;
       
       // Check if box would stop before reaching transition/end
       float stopping_distance = CalculateStoppingDistanceOnPlatform(current_velocity, *current_platform);
       
       if (has_transition) {
           distance_traveled = std::min(distance_to_transition, stopping_distance);
           if (distance_traveled >= stopping_distance) {
               // Box stops before transition
               end_position = Vector2Add(current_position, Vector2Scale(Vector2Normalize(Vector2Subtract(transition_point, current_position)), distance_traveled));
               has_transition = false; // Won't reach next platform
           } else {
               // Box reaches transition point
               end_position = transition_point;
           }
       } else {
           // No transition, move towards right end of platform
           distance_traveled = std::min(distance_to_right_end, stopping_distance);
           end_position = Vector2Add(current_position, Vector2Scale(Vector2Normalize(Vector2Subtract(platform_right_end, current_position)), distance_traveled));
       }
       
       // Calculate final velocity after traveling this distance using improved physics
       float final_velocity = CalculateFinalVelocity(current_velocity, *current_platform, distance_traveled);
       
       // Set up the segment
       segment.end_position = end_position;
       segment.distance = distance_traveled;
       trajectory_segments.push_back(segment);
       
       // Check if we should stop
       if (!has_transition || final_velocity <= 0.001f || distance_traveled >= stopping_distance) {
           final_ghost_position = end_position;
           if (final_velocity <= 0.001f || distance_traveled >= stopping_distance) {
               std::cout << "Box stops on platform " << segment_count << std::endl;
           } else {
               std::cout << "Box reaches end of connected platforms at segment " << segment_count << std::endl;
           }
           break;
       }
       
       // Continue to next platform
       current_position = end_position;
       current_velocity = final_velocity;
       current_platform = next_platform;
       segment_count++;
   }
}

const Platform* Box::FindNextConnectedPlatform(const Platform& current_platform, const std::vector<Platform>& platforms) {
   float tolerance = 30.0f;
   const Platform* best_platform = nullptr;
   float min_distance = std::numeric_limits<float>::max();
   
   for (const auto& platform : platforms) {
       if (&platform == &current_platform) continue;
       
       // Check if platforms intersect or are very close
       Vector2 intersection;
       if (CheckCollisionLines(current_platform.top_left, current_platform.top_right,
                              platform.top_left, platform.top_right, &intersection)) {
           // Direct intersection - this is definitely connected
           return &platform;
       }
       
       // Check proximity between platform endpoints
       float dist1 = Vector2Distance(current_platform.top_right, platform.top_left);
       float dist2 = Vector2Distance(current_platform.top_right, platform.top_right);
       float dist3 = Vector2Distance(current_platform.top_left, platform.top_left);
       float dist4 = Vector2Distance(current_platform.top_left, platform.top_right);
       
       float min_dist = std::min({dist1, dist2, dist3, dist4});
       
       if (min_dist < tolerance && min_dist < min_distance) {
           min_distance = min_dist;
           best_platform = &platform;
       }
   }
   
   return best_platform;
}

void Box::DrawTwoLineCollisionDebug(const std::vector<Platform>& platforms) {
   // Get the two vertical lines from the box's bottom corners
   std::vector<LineSegment> lines = CollisionUtils::GetBoxBottomLines(position, size, rotation);
   
   // Check collision first to get the collision points
   TwoLineCollisionInfo collisionInfo = CollisionUtils::CheckTwoLineCollision(lines, platforms);
   
   // Draw the two lines from box corners to platform surface
   for (size_t i = 0; i < lines.size(); i++) {
       Color lineColor = (i == 0) ? RED : BLUE;
       
       // Always draw line from box corner to platform surface
       Vector2 endPoint = lines[i].end; // Default to full line
       
       if (collisionInfo.hasCollision) {
           // Use the collision point as the end point
           endPoint = collisionInfo.collisionPoint;
       } else {
           // Find the closest platform point for this line
           for (const auto& platform : platforms) {
               Vector2 closestPoint = CollisionUtils::GetClosestPointOnPlatform(lines[i], platform);
               float distance = Vector2Distance(lines[i].start, closestPoint);
               
               // If this platform is close and below the line, use it
               if (lines[i].start.y > closestPoint.y && distance < 100.0f) {
                   endPoint = closestPoint;
                   break;
               }
           }
       }
       
       // Draw line from box corner to platform surface
       DrawLineEx(lines[i].start, endPoint, 2.0f, lineColor);
       
       // Draw a small circle at the start of each line
       DrawCircleV(lines[i].start, 3.0f, lineColor);
   }
   
   if (collisionInfo.hasCollision) {
       // Draw collision point
       DrawCircleV(collisionInfo.collisionPoint, 5.0f, GREEN);
       
       // Draw distance text
       Vector2 textPos = {collisionInfo.collisionPoint.x + 10, collisionInfo.collisionPoint.y - 10};
       DrawText(TextFormat("Dist: %.2f", collisionInfo.distanceToPlatform), textPos.x, textPos.y, 14, WHITE);
       DrawText(TextFormat("Platform: %d", collisionInfo.platformId), textPos.x, textPos.y + 15, 14, WHITE);
   }
}

bool Box::IsPointOnPlatform(Vector2 point, const Platform& platform) {
   // Check if point is close to the platform's top surface
   float tolerance = 25.0f; // Increased tolerance for better detection
   
   // Get platform top line
   Vector2 line_start = platform.top_left;
   Vector2 line_end = platform.top_right;
   
   // Calculate distance from point to line
   float line_length = Vector2Distance(line_start, line_end);
   if (line_length == 0) return false;
   
   Vector2 line_vec = Vector2Subtract(line_end, line_start);
   Vector2 point_vec = Vector2Subtract(point, line_start);
   
   float t = Vector2DotProduct(point_vec, line_vec) / (line_length * line_length);
   t = fmax(0.0f, fmin(1.0f, t)); // Clamp to line segment
   
   Vector2 closest_point = Vector2Add(line_start, Vector2Scale(line_vec, t));
   float distance = Vector2Distance(point, closest_point);
   
   return distance <= tolerance;
}

float Box::GetDistanceToEndOfPlatform(Vector2 start_pos, const Platform& platform) {
   Vector2 platform_direction = GetPlatformDirection(platform);
   Vector2 platform_end;
   
   // Determine which end of the platform we're heading towards
   float dot_to_right = Vector2DotProduct(Vector2Subtract(platform.top_right, start_pos), platform_direction);
   float dot_to_left = Vector2DotProduct(Vector2Subtract(platform.top_left, start_pos), platform_direction);
   
   if (dot_to_right > 0 && dot_to_right > dot_to_left) {
       platform_end = platform.top_right;
   } else {
       platform_end = platform.top_left;
   }
   
   return Vector2Distance(start_pos, platform_end);
}

Vector2 Box::GetPlatformDirection(const Platform& platform) {
   Vector2 direction = Vector2Subtract(platform.top_right, platform.top_left);
   return Vector2Normalize(direction);
}

float Box::CalculateFinalVelocity(float initial_velocity, const Platform& platform, float distance) {
   float slope_angle = platform.rotation * DEG2RAD;
   
   // PURE KINEMATICS: acceleration = g*sin(θ) - μ*g*cos(θ)
   float gravity_component = gravity * sin(slope_angle);
   float friction_component = mu_kinetic * gravity * cos(fabs(slope_angle));
   float net_acceleration = gravity_component - friction_component;
   
   // Use kinematic equation: v² = u² + 2as
   float final_velocity_squared = initial_velocity * initial_velocity + 2.0f * net_acceleration * distance;
   
   if (final_velocity_squared <= 0) {
       return 0.0f; // Box stops
   }
   
   return sqrt(final_velocity_squared);
}


float Box::CalculateStoppingDistanceOnPlatform(float initial_velocity, const Platform& platform) {
   float slope_angle = platform.rotation * DEG2RAD;
   
   // If no initial velocity, box won't move
   if (initial_velocity <= 0.001f) {
       return 0.0f;
   }
   
   // PURE KINEMATICS: Calculate acceleration and use v² = u² + 2as
   float gravity_component = gravity * sin(slope_angle);
   float friction_component = mu_kinetic * gravity * cos(fabs(slope_angle));
   float net_acceleration = gravity_component - friction_component;
   
   // For horizontal platforms (or when friction > gravity component)
   if (abs(platform.rotation) <= 5.0f || net_acceleration <= 0) {
       // Only friction decelerates: a = -μg
       // 0 = u² + 2as → s = -u²/(2a) = u²/(2μg)
       float deceleration = mu_kinetic * gravity;
       return (initial_velocity * initial_velocity) / (2.0f * deceleration);
   } else {
       // Box will accelerate down the slope and won't stop on this platform
       return std::numeric_limits<float>::max();
   }
}

float Box::CalculateDistanceToConnection(Vector2 start_pos, const Platform& current_platform, const Platform& next_platform) {
   // Find the connection point between current and next platform
   Vector2 connection_point;
   
   // Check intersection between platform edges
   if (CheckCollisionLines(current_platform.top_left, current_platform.top_right,
                          next_platform.top_left, next_platform.top_right, &connection_point)) {
       return Vector2Distance(start_pos, connection_point);
   }
   
   // If no direct intersection, find closest connection
   float dist_to_next_left = Vector2Distance(current_platform.top_right, next_platform.top_left);
   float dist_to_next_right = Vector2Distance(current_platform.top_left, next_platform.top_right);
   
   if (dist_to_next_left < dist_to_next_right) {
       return Vector2Distance(start_pos, current_platform.top_right);
   } else {
       return Vector2Distance(start_pos, current_platform.top_left);
   }
}

const Platform* Box::FindNextPlatform(Vector2 current_pos, const std::vector<Platform>& platforms, const Platform* current_platform) {
   float tolerance = 30.0f;
   const Platform* closest_platform = nullptr;
   float min_distance = std::numeric_limits<float>::max();
   
   // Get the direction we're moving on current platform
   Vector2 movement_direction = GetPlatformDirection(*current_platform);
   
   for (const auto& platform : platforms) {
       if (&platform == current_platform) continue;
       
       // Check both ends of the next platform
       float dist_to_left = Vector2Distance(current_platform->top_right, platform.top_left);   // Changed . to ->
       float dist_to_right = Vector2Distance(current_platform->top_left, platform.top_right);  // Changed . to ->
       float min_dist = std::min(dist_to_left, dist_to_right);
       
       if (min_dist < tolerance && min_dist < min_distance) {
           min_distance = min_dist;
           closest_platform = &platform;
       }
   }
   
   return closest_platform;
}

Vector2 Box::GetRightEndOfPlatform(const Platform& platform) {
   // Always return the rightmost point of the platform
   if (platform.top_left.x > platform.top_right.x) {
       return platform.top_left;
   } else {
       return platform.top_right;
   }
}

const Platform* Box::FindNextPlatformToRight(const Platform& current_platform, const std::vector<Platform>& platforms) {
    std::cout << "\n=== FindNextPlatformToRight DEBUG ===" << std::endl;
    std::cout << "Current platform ID: " << current_platform.id << std::endl;
    std::cout << "Current platform rotation: " << current_platform.rotation << std::endl;
    
    Vector2 current_right_end = GetRightEndOfPlatform(current_platform);
    std::cout << "Current platform right end: (" << current_right_end.x << ", " << current_right_end.y << ")" << std::endl;
    
    // Find the platform with the next highest ID
    const Platform* next_platform = nullptr;
    int target_id = current_platform.id + 1;
    
    for (size_t i = 0; i < platforms.size(); i++) {
        const auto& platform = platforms[i];
        
        std::cout << "\nChecking platform " << platform.id << " (rotation: " << platform.rotation << ")" << std::endl;
        
        if (platform.id == target_id) {
            std::cout << "Found next platform by ID: " << platform.id << std::endl;
            next_platform = &platform;
            break;
        }
    }
    
    if (next_platform) {
        std::cout << "=== SELECTED: Platform " << next_platform->id << " ===" << std::endl;
        return next_platform;
    }
    
    std::cout << "=== NO PLATFORM FOUND (no platform with ID " << target_id << ") ===" << std::endl;
    return nullptr;
}

// ================================
// NEW REFERENCE FRAME-BASED TRAJECTORY CALCULATION
// ================================
                        
// === NEW: Analytic per-platform reference frame trajectory ===
// Each platform is its own reference frame. Between platforms we either:
//   (a) transfer at a shared endpoint (connected), or
//   (b) follow a projectile arc to the first forward intersection.
// Inelastic impacts: normal component is discarded; tangential component is preserved.
// On-platform motion uses your friction-only kinematics:
//   a_t = g*sin(theta) - mu*g*cos(theta), and v^2 = u^2 + 2 a s.

namespace {
    // Analytic intersection of projectile with a line segment.
    // Projectile (downward +y gravity): p(t) = p0 + v0*t + 0.5*(0, g)*t^2
    // Segment: l(s) = L0 + s*(L1-L0), 0<=s<=1
    inline bool SolveProjectileSegmentIntersection(Vector2 p0, Vector2 v0, float g,
                                                   Vector2 L0, Vector2 L1,
                                                   float& t_hit, float& s_hit, Vector2& hit)
    {
        Vector2 U = { L1.x - L0.x, L1.y - L0.y };
        if (fabsf(U.x) > 1e-6f) {
            // Eliminate s via x
            float A = 0.5f * g;
            float B = (v0.y - (U.y / U.x) * v0.x);
            float C = (p0.y - L0.y - (U.y / U.x) * (p0.x - L0.x));
            float disc = B*B - 4*A*C;
            if (disc < 0) return false;
            float sqrt_disc = sqrtf(disc);
            float t1 = (-B - sqrt_disc) / (2*A);
            float t2 = (-B + sqrt_disc) / (2*A);
            float tCandidate = 1e30f;
            if (t1 > 1e-5f) tCandidate = t1;
            if (t2 > 1e-5f && t2 < tCandidate) tCandidate = t2;
            if (tCandidate == 1e30f) return false;
            float x = p0.x + v0.x * tCandidate;
            float s = (x - L0.x) / U.x;
            if (s < -1e-4f || s > 1.0001f) return false;
            t_hit = tCandidate; s_hit = s;
            hit = { L0.x + s * U.x, L0.y + s * U.y };
            return true;
        } else if (fabsf(U.y) > 1e-6f) {
            // Vertical-ish segment: eliminate s via y (allow small x error)
            float A = 0.5f * g, B = v0.y;
            auto smallest_positive_root = [&](float C)->float {
                float disc = B*B - 4*A*C;
                if (disc < 0) return 1e30f;
                float sd = sqrtf(disc);
                float r1 = (-B - sd)/(2*A), r2 = (-B + sd)/(2*A);
                float t = 1e30f;
                if (r1 > 1e-5f) t = r1;
                if (r2 > 1e-5f && r2 < t) t = r2;
                return t;
            };
            float C0 = (p0.y - L0.y);
            float C1 = (p0.y - L1.y);
            float tA = smallest_positive_root(C0);
            float tB = smallest_positive_root(C1);
            float tCandidate = std::min(tA, tB);
            if (tCandidate == 1e30f) return false;
            float y = p0.y + v0.y * tCandidate + 0.5f * g * tCandidate * tCandidate;
            float s = (y - L0.y) / U.y;
            float x = p0.x + v0.x * tCandidate;
            if (s < -1e-4f || s > 1.0001f) return false;
            if (fabsf((L0.x + s*U.x) - x) > 2.0f) return false;
            t_hit = tCandidate; s_hit = s;
            hit = { L0.x + s * U.x, L0.y + s * U.y };
            return true;
        }
        return false;
    }

    inline Vector2 UnitTangent(const Platform& p) {
        Vector2 t = Vector2Normalize(Vector2Subtract(p.top_right, p.top_left));
        if (t.x < 0) t = Vector2Scale(t, -1.0f); // forward = to the right
        return t;
    }
    inline float SignedDistanceAlong(const Platform& p, Vector2 a, Vector2 b) {
        Vector2 t = UnitTangent(p);
        return Vector2DotProduct(Vector2Subtract(b, a), t);
    }
    inline Vector2 MoveAlong(const Platform& p, Vector2 a, float s) {
        Vector2 t = UnitTangent(p);
        return Vector2Add(a, Vector2Scale(t, s));
    }
}

void Box::CalculateMultiReferenceFrameTrajectory(const std::vector<Platform>& platforms) {
    trajectory_segments.clear();
    projectile_trajectory_points.clear();
    multi_platform_ghost_calculated = false;

    if (platforms.empty()) {
        final_ghost_position = position;
        multi_platform_ghost_calculated = true;
        return;
    }

    // Sort platforms left->right (stable) so "forward" means increasing x
    std::vector<const Platform*> sorted;
    sorted.reserve(platforms.size());
    for (auto& p : platforms) sorted.push_back(&p);
    std::sort(sorted.begin(), sorted.end(), [](const Platform* a, const Platform* b){
        float ax = std::min(a->top_left.x, a->top_right.x);
        float bx = std::min(b->top_left.x, b->top_right.x);
        if (ax == bx) return a->id < b->id;
        return ax < bx;
    });

    // Find supporting platform at the start point
    auto find_support = [&](Vector2 pt)->const Platform* {
        const Platform* best = nullptr; float best_d = 1e30f;
        for (const Platform* p : sorted) {
            Vector2 a = p->top_left, b = p->top_right;
            Vector2 ab = Vector2Subtract(b, a);
            float L2 = ab.x*ab.x + ab.y*ab.y; if (L2 < 1e-4f) continue;
            float t = Vector2DotProduct(Vector2Subtract(pt, a), ab) / L2;
            t = std::clamp(t, 0.0f, 1.0f);
            Vector2 closest = Vector2Add(a, Vector2Scale(ab, t));
            float d = Vector2Distance(pt, closest);
            if (d < best_d && d <= 24.0f) { best_d = d; best = p; }
        }
        return best;
    };

    Vector2 cur_pos  = has_prediction_start ? prediction_start_position : position;
    const Platform* cur_plat = find_support(cur_pos);
    if (!cur_plat) {
        final_ghost_position = cur_pos;
        multi_platform_ghost_calculated = true;
        return;
    }

    // Project current velocity onto platform tangent for initial speed
    float cur_speed = fabsf(Vector2DotProduct(velocity, UnitTangent(*cur_plat)));

    int guard = 0;
    while (cur_plat && guard++ < 64) {
        // Choose forward end of current platform
        Vector2 endA = cur_plat->top_left, endB = cur_plat->top_right;
        Vector2 forward_end = (endA.x > endB.x) ? endA : endB;
        if (UnitTangent(*cur_plat).x < 0) forward_end = (endA.x < endB.x) ? endA : endB;

        // Find earlier "connected" intersection ahead along current platform
        Vector2 clamped_end = forward_end;
        const Platform* connected_next = nullptr;
        for (const Platform* cand : sorted) {
            if (cand == cur_plat) continue;
            if (std::min(cand->top_left.x, cand->top_right.x) < std::min(cur_plat->top_left.x, cur_plat->top_right.x))
                continue; // forward only
            Vector2 inter;
            if (CheckCollisionLines(cur_plat->top_left, cur_plat->top_right, cand->top_left, cand->top_right, &inter)) {
                float ahead = SignedDistanceAlong(*cur_plat, cur_pos, inter);
                if (ahead > 1e-4f) {
                    float current_end_ahead = fabsf(SignedDistanceAlong(*cur_plat, cur_pos, clamped_end));
                    if (ahead < current_end_ahead) {
                        clamped_end = inter;
                        connected_next = cand;
                    }
                }
            }
        }

        // Distances along the current platform
        float dist_to_clamped = fabsf(SignedDistanceAlong(*cur_plat, cur_pos, clamped_end));
        float stop_dist       = CalculateStoppingDistanceOnPlatform(cur_speed, *cur_plat);

        // Case A: we stop on this platform before end
        if (stop_dist <= dist_to_clamped + 1e-6f) {
            Vector2 stop_pos = MoveAlong(*cur_plat, cur_pos, stop_dist);
            trajectory_segments.push_back({cur_pos, stop_pos, cur_plat, stop_dist});
            final_ghost_position = stop_pos;
            multi_platform_ghost_calculated = true;
            return;
        }

        // We reach clamped_end with exit speed
        float exit_speed = CalculateFinalVelocity(cur_speed, *cur_plat, dist_to_clamped);
        trajectory_segments.push_back({cur_pos, clamped_end, cur_plat, dist_to_clamped});

        // Case B1: connected next platform — instant transfer, keep tangential speed
        if (connected_next) {
            cur_pos   = clamped_end;
            cur_plat  = connected_next;
            cur_speed = exit_speed;
            continue;
        }

        // Case B2: gap — projectile to first forward platform we hit (analytic)
        Vector2 launch_pos = clamped_end;
        Vector2 launch_vel = Vector2Scale(UnitTangent(*cur_plat), exit_speed);

        float best_t = 1e30f; const Platform* best_plat = nullptr; Vector2 best_hit{}; float best_s = 0.0f;
        for (const Platform* cand : sorted) {
            if (cand == cur_plat) continue;
            if (std::min(cand->top_left.x, cand->top_right.x) < std::min(cur_plat->top_left.x, cur_plat->top_right.x) - 1.0f)
                continue;
            float t_hit, s_hit; Vector2 hit;
            if (SolveProjectileSegmentIntersection(launch_pos, launch_vel, gravity,
                                                   cand->top_left, cand->top_right,
                                                   t_hit, s_hit, hit)) {
                if (t_hit > 1e-5f && t_hit < best_t && (hit.x >= launch_pos.x - 1e-3f)) {
                    best_t = t_hit; best_plat = cand; best_hit = hit; best_s = s_hit;
                }
            }
        }

        if (!best_plat) {
            // No landing found: simulate a short arc for display AND count its distance
            projectile_trajectory_points.clear();
            Vector2 cur = launch_pos, v = launch_vel;
            float dt = 0.02f, T = 0.6f;
            projectile_trajectory_points.push_back(cur);
            for (float tacc=0; tacc<T; tacc+=dt) {
                v.y += gravity * dt;
                cur = Vector2Add(cur, Vector2Scale(v, dt));
                projectile_trajectory_points.push_back(cur);
            }

            // Record this "partial jump" as a trajectory segment with arc length
            TrajectorySegment arc_seg;
            arc_seg.start_position = launch_pos;
            arc_seg.end_position   = cur;
            arc_seg.platform       = nullptr;
            arc_seg.distance       = ArcLenProjectile(launch_vel, gravity, T);
            trajectory_segments.push_back(arc_seg);

            final_ghost_position = cur;
            multi_platform_ghost_calculated = true;
            return;
        }

        // Record a smooth arc for drawing
        projectile_trajectory_points.clear();
        int steps = 24;
        for (int i=0; i<=steps; ++i) {
            float t = best_t * (float(i)/steps);
            Vector2 pt = {
                launch_pos.x + launch_vel.x * t,
                launch_pos.y + launch_vel.y * t + 0.5f * gravity * t * t
            };
            projectile_trajectory_points.push_back(pt);
        }

        // Also record the jump as a segment with its true arc length
        {
            TrajectorySegment arc_seg;
            arc_seg.start_position = launch_pos;
            arc_seg.end_position   = best_hit;
            arc_seg.platform       = nullptr; // projectile arc
            arc_seg.distance       = ArcLenProjectile(launch_vel, gravity, best_t);
            trajectory_segments.push_back(arc_seg);
        }

        // Inelastic landing: keep tangential component along the landing platform
        Vector2 v_impact = { launch_vel.x, launch_vel.y + gravity * best_t };
        Vector2 t_new = UnitTangent(*best_plat);
        float speed_after = fabsf(Vector2DotProduct(v_impact, t_new));

        // Continue on landing platform
        cur_pos   = best_hit;
        cur_plat  = best_plat;
        cur_speed = speed_after;
    }

    // Safety
    final_ghost_position = cur_pos;
    multi_platform_ghost_calculated = true;
}




// NEW: Find connected platform groups
std::vector<std::vector<const Platform*>> Box::FindPlatformGroups(const std::vector<Platform>& platforms) {
   std::vector<std::vector<const Platform*>> groups;
   std::set<const Platform*> processed;
   
   for (const auto& platform : platforms) {
       if (processed.find(&platform) != processed.end()) continue;
       
       // Start a new group
       std::vector<const Platform*> current_group;
       std::vector<const Platform*> to_process = { &platform };
       
       while (!to_process.empty()) {
           const Platform* current = to_process.back();
           to_process.pop_back();
           
           if (processed.find(current) != processed.end()) continue;
           
           processed.insert(current);
           current_group.push_back(current);
           
           // Find connected platforms
           for (const auto& other : platforms) {
               if (processed.find(&other) != processed.end()) continue;
               
               // Check if platforms are connected (intersection or close proximity)
               Vector2 intersection;
               bool connected = CheckCollisionLines(current->top_left, current->top_right,
                                                  other.top_left, other.top_right, &intersection);
               
               if (!connected) {
                   // Check proximity
                   Vector2 current_right = GetRightEndOfPlatform(*current);
                   Vector2 other_left = (other.top_left.x < other.top_right.x) ? other.top_left : other.top_right;
                   float distance = Vector2Distance(current_right, other_left);
                   
                   if (distance < 50.0f) { // Close enough to be connected
                       connected = true;
                   }
               }
               
               if (connected) {
                   to_process.push_back(&other);
               }
           }
       }
       
       if (!current_group.empty()) {
           // Sort platforms in group by x position
           std::sort(current_group.begin(), current_group.end(),
               [](const Platform* a, const Platform* b) {
                   float a_left = std::min(a->top_left.x, a->top_right.x);
                   float b_left = std::min(b->top_left.x, b->top_right.x);
                   return a_left < b_left;
               });
           
           groups.push_back(current_group);
       }
   }
   
   std::cout << "Found " << groups.size() << " platform groups" << std::endl;
   return groups;
}

// NEW: Process trajectory through a single platform group
bool Box::ProcessPlatformGroup(const std::vector<const Platform*>& group, Vector2 start_pos, float initial_speed, Vector2& end_pos, float& end_speed) {
   Vector2 current_pos = start_pos;
   float current_speed = initial_speed;
   
   for (const Platform* platform : group) {
       // Calculate trajectory along this platform
       Vector2 platform_end = GetRightEndOfPlatform(*platform);
       float distance_on_platform = Vector2Distance(current_pos, platform_end);
       
       // Check if box stops on this platform
       float stopping_distance = CalculateStoppingDistanceOnPlatform(current_speed, *platform);
       
       if (stopping_distance <= distance_on_platform) {
           // Box stops on this platform
           Vector2 platform_direction = GetPlatformDirection(*platform);
           end_pos = Vector2Add(current_pos, Vector2Scale(platform_direction, stopping_distance));
           end_speed = 0.0f;
           
           // Add trajectory segment
           TrajectorySegment segment;
           segment.start_position = current_pos;
           segment.end_position = end_pos;
           segment.platform = platform;
           segment.distance = stopping_distance;
           trajectory_segments.push_back(segment);
           
           return false; // Box stopped
       } else {
           // Box reaches end of platform
           float final_speed = CalculateFinalVelocity(current_speed, *platform, distance_on_platform);
           
           // Add trajectory segment
           TrajectorySegment segment;
           segment.start_position = current_pos;
           segment.end_position = platform_end;
           segment.platform = platform;
           segment.distance = distance_on_platform;
           trajectory_segments.push_back(segment);
           
           current_pos = platform_end;
           current_speed = final_speed;
       }
   }
   
   end_pos = current_pos;
   end_speed = current_speed;
   return true; // Successfully processed entire group
}

// NEW: Calculate projectile motion to next platform group
Vector2 Box::CalculateProjectileToGroup(Vector2 launch_pos, float launch_speed, const std::vector<const Platform*>& target_group) {
   // Use the existing projectile motion calculation
   // Calculate launch velocity (horizontal for now, can be improved)
   Vector2 launch_velocity = { launch_speed, 0.0f };
   
   // Convert platform group to vector for existing function
   std::vector<Platform> temp_platforms;
   for (const Platform* platform : target_group) {
       temp_platforms.push_back(*platform);
   }
   
   // Use existing projectile calculation
   Vector2 landing_position = CalculateProjectileTrajectory(launch_pos, launch_velocity, temp_platforms);
   
   return landing_position;
}

// NEW: Calculate projectile motion trajectory
Vector2 Box::CalculateProjectileTrajectory(Vector2 launch_position, Vector2 launch_velocity, const std::vector<Platform>& platforms) {
    // Projectile motion parameters
    float dt = 0.02f; // Smaller time step for more accurate trajectory
    Vector2 current_pos = launch_position;
    Vector2 current_vel = launch_velocity;
    float max_time = 10.0f; // Safety cap
    float time = 0.0f;

    // Store projectile trajectory points for drawing the arc
    std::vector<Vector2> projectile_path;
    projectile_path.push_back(current_pos);

    while (time < max_time) {
        // Previous position for segment/line intersection
        Vector2 prev_pos = current_pos;

        // Update projectile state (kinematics only)
        current_vel.y += gravity * dt;
        current_pos = Vector2Add(current_pos, Vector2Scale(current_vel, dt));

        // Check intersection with each candidate platform's top edge
        for (const auto& platform : platforms) {
            Vector2 intersection;
            bool crosses = CheckCollisionLines(prev_pos, current_pos, platform.top_left, platform.top_right, &intersection);

            if (crosses) {
                // Ensure we're landing from above (downward motion)
                if (prev_pos.y <= intersection.y && current_vel.y > 0) {
                    // Clamp the last point to the exact intersection
                    projectile_path.push_back(intersection);

                    // Record the projectile segment only
                    TrajectorySegment projectile_segment;
                    projectile_segment.start_position = launch_position;
                    projectile_segment.end_position   = intersection;
                    projectile_segment.platform       = nullptr; // arc segment
                    projectile_segment.distance       = Vector2Distance(launch_position, intersection);
                    trajectory_segments.push_back(projectile_segment);

                    // Expose arc points for drawing
                    projectile_trajectory_points = projectile_path;

                    // Return the LANDING POINT only; caller will handle post-landing sliding
                    return intersection;
                }
            }
        }

        // Also handle world bounds as a landing/finalization case
        if (current_pos.y >= GetScreenHeight() - 10) {
            Vector2 ground_collision = { current_pos.x, (float)GetScreenHeight() - 10 };
            projectile_path.push_back(ground_collision);

            TrajectorySegment projectile_segment;
            projectile_segment.start_position = launch_position;
            projectile_segment.end_position   = ground_collision;
            projectile_segment.platform       = nullptr;
            projectile_segment.distance       = Vector2Distance(launch_position, ground_collision);
            trajectory_segments.push_back(projectile_segment);

            projectile_trajectory_points = projectile_path;
            return ground_collision;
        }

        if (current_pos.x < 0 || current_pos.x > GetScreenWidth()) {
            // Off-screen horizontally — end arc at current_pos
            projectile_path.push_back(current_pos);

            TrajectorySegment projectile_segment;
            projectile_segment.start_position = launch_position;
            projectile_segment.end_position   = current_pos;
            projectile_segment.platform       = nullptr;
            projectile_segment.distance       = Vector2Distance(launch_position, current_pos);
            trajectory_segments.push_back(projectile_segment);

            projectile_trajectory_points = projectile_path;
            return current_pos;
        }

        // Accumulate arc path and advance time
        projectile_path.push_back(current_pos);
        time += dt;
    }

    // Fallback: no hit within max_time — return last simulated pos
    projectile_trajectory_points = projectile_path;
    return current_pos;
}


// NEW: Calculate trajectory after landing on a platform
Vector2 Box::CalculatePostLandingTrajectory(Vector2 landing_position, Vector2 landing_velocity, const Platform& landing_platform) {
   // Calculate the velocity component along the platform surface
   float platform_angle = landing_platform.rotation * DEG2RAD;
   Vector2 platform_direction = { cos(platform_angle), sin(platform_angle) };
   
   // Project landing velocity onto platform direction to get speed along platform
   float speed_along_platform = Vector2DotProduct(landing_velocity, platform_direction);
   
   // Make sure we're moving in the positive platform direction
   if (speed_along_platform < 0) {
       speed_along_platform = fabs(speed_along_platform);
   }
   
   // Calculate deceleration on the platform (same physics as normal platform movement)
   float platform_slope_angle = landing_platform.rotation * DEG2RAD;
   float gravity_component = gravity * sin(platform_slope_angle);
   float friction_component = mu_kinetic * gravity * cos(fabs(platform_slope_angle));
   float net_deceleration = friction_component - gravity_component;
   
   // If net deceleration is negative or zero, box will accelerate or maintain speed
   if (net_deceleration <= 0) {
       // Box continues to end of platform
       Vector2 platform_end = GetRightEndOfPlatform(landing_platform);
       
       // Add platform segment
       TrajectorySegment platform_segment;
       platform_segment.start_position = landing_position;
       platform_segment.end_position = platform_end;
       platform_segment.platform = &landing_platform;
       platform_segment.distance = Vector2Distance(landing_position, platform_end);
       trajectory_segments.push_back(platform_segment);
       
       return platform_end;
   }
   
   // Calculate stopping distance using kinematic equation: v² = u² - 2as
   // Final velocity = 0, so: 0 = u² - 2as → s = u²/(2a)
   float stopping_distance = (speed_along_platform * speed_along_platform) / (2.0f * net_deceleration);
   
   // Calculate stopping position along platform
   Vector2 platform_direction_normalized = Vector2Normalize(platform_direction);
   Vector2 stopping_position = Vector2Add(landing_position, Vector2Scale(platform_direction_normalized, stopping_distance));
   
   // Check if stopping position is beyond the end of the platform
   Vector2 platform_end = GetRightEndOfPlatform(landing_platform);
   float distance_to_end = Vector2Distance(landing_position, platform_end);
   
   Vector2 final_position;
   if (stopping_distance >= distance_to_end) {
       // Box doesn't stop on this platform - goes to the end
       final_position = platform_end;
       
       // Calculate remaining velocity at platform end
       float distance_traveled = distance_to_end;
       float velocity_lost_squared = 2.0f * net_deceleration * distance_traveled;
       float remaining_velocity_squared = speed_along_platform * speed_along_platform - velocity_lost_squared;
       float remaining_velocity = (remaining_velocity_squared > 0) ? sqrt(remaining_velocity_squared) : 0.0f;
       
       // Add platform segment
       TrajectorySegment platform_segment;
       platform_segment.start_position = landing_position;
       platform_segment.end_position = final_position;
       platform_segment.platform = &landing_platform;
       platform_segment.distance = distance_traveled;
       trajectory_segments.push_back(platform_segment);
       
       // If there's remaining velocity, could potentially jump to another platform
       // For now, just stop at the end of this platform
       return final_position;
   } else {
       // Box stops on this platform
       final_position = stopping_position;
       
       // Add platform segment
       TrajectorySegment platform_segment;
       platform_segment.start_position = landing_position;
       platform_segment.end_position = final_position;
       platform_segment.platform = &landing_platform;
       platform_segment.distance = stopping_distance;
       trajectory_segments.push_back(platform_segment);
       
       return final_position;
   }
}

// Helper function to check if trajectory segment crosses platform
bool Box::CheckTrajectoryPlatformCollision(Vector2 start_pos, Vector2 end_pos, const Platform& platform) {
   // Check if the trajectory line segment intersects with the platform top surface
   Vector2 intersection;
   return CheckCollisionLines(start_pos, end_pos, platform.top_left, platform.top_right, &intersection);
}

// Helper function to find exact collision point on platform
Vector2 Box::FindTrajectoryCollisionPoint(Vector2 start_pos, Vector2 end_pos, const Platform& platform) {
   Vector2 intersection;
   if (CheckCollisionLines(start_pos, end_pos, platform.top_left, platform.top_right, &intersection)) {
       return intersection;
   }
   // Fallback to end position if no intersection found
   return end_pos;
}

std::vector<Box::PlatformPair> Box::FindConnectedPlatformPairs(const std::vector<Platform>& platforms, Vector2 start_position) {
   std::vector<PlatformPair> reference_frames;
   
   // Find the starting platform
   const Platform* current_platform = nullptr;
   for (const auto& platform : platforms) {
       if (IsPointOnPlatform(start_position, platform)) {
           current_platform = &platform;
           break;
       }
   }
   
   if (!current_platform) {
       std::cout << "No starting platform found at position (" << start_position.x << ", " << start_position.y << ")" << std::endl;
       return reference_frames;
   }
   
   std::cout << "Starting platform found with rotation: " << current_platform->rotation << std::endl;
   
   // Create reference frames: each frame is slope + horizontal pair
   const Platform* working_platform = current_platform;
   
   for (int frame = 0; frame < 5 && working_platform; frame++) { // Max 5 reference frames
       PlatformPair reference_frame;
       
       // Each reference frame consists of current platform + next platform
       reference_frame.slope_platform = working_platform;
       reference_frame.horizontal_platform = FindNextPlatformToRight(*working_platform, platforms);
       
       if (reference_frame.horizontal_platform) {
           // Find intersection between the two platforms in this reference frame
           reference_frame.has_intersection = CheckCollisionLines(
               reference_frame.slope_platform->top_left, reference_frame.slope_platform->top_right,
               reference_frame.horizontal_platform->top_left, reference_frame.horizontal_platform->top_right,
               &reference_frame.transition_point
           );
           
           if (!reference_frame.has_intersection) {
               // No direct intersection - use connection point
               Vector2 first_right = GetRightEndOfPlatform(*reference_frame.slope_platform);
               Vector2 second_left = (reference_frame.horizontal_platform->top_left.x < reference_frame.horizontal_platform->top_right.x) ? 
                                  reference_frame.horizontal_platform->top_left : reference_frame.horizontal_platform->top_right;
               
               reference_frame.transition_point = Vector2Scale(Vector2Add(first_right, second_left), 0.5f);
               reference_frame.has_intersection = true;
           }
           
           reference_frames.push_back(reference_frame);
           std::cout << "Reference Frame " << frame + 1 << ": Platform rotation=" << reference_frame.slope_platform->rotation 
                     << " + Platform rotation=" << reference_frame.horizontal_platform->rotation << std::endl;
           
           // IMPORTANT: Skip to the platform AFTER this reference frame (skip the horizontal_platform)
           working_platform = FindNextPlatformToRight(*reference_frame.horizontal_platform, platforms);
       } else {
           // Last platform - create single-platform reference frame
           reference_frame.horizontal_platform = nullptr;
           reference_frame.has_intersection = false;
           reference_frames.push_back(reference_frame);
           std::cout << "Final Reference Frame " << frame + 1 << ": Single platform rotation=" << reference_frame.slope_platform->rotation << std::endl;
           break;
       }
   }
   
   std::cout << "Created " << reference_frames.size() << " reference frames total" << std::endl;
   return reference_frames;
}

bool Box::CalculateSingleReferenceFrame(const PlatformPair& pair, Vector2 start_pos, float initial_velocity, Vector2& end_pos, float& final_velocity) {
   if (!pair.slope_platform) {
       return false;
   }
   
   // Case 1: Two-platform reference frame
   if (pair.horizontal_platform) {
       std::cout << "Calculating two-platform reference frame" << std::endl;
       
       // Calculate trajectory along slope
       Vector2 slope_end_pos;
       float slope_final_velocity;
       
       if (pair.has_intersection) {
           slope_end_pos = pair.transition_point;
       } else {
           slope_end_pos = GetRightEndOfPlatform(*pair.slope_platform);
       }
       
       // Calculate distance along slope
       float slope_distance = GetDistanceAlongSlope(start_pos, slope_end_pos, *pair.slope_platform);
       
       // Calculate final velocity after slope using physics: v² = u² + 2as
       float slope_acceleration = gravity * sin(pair.slope_platform->rotation * DEG2RAD);
       float velocity_squared = initial_velocity * initial_velocity + 2.0f * slope_acceleration * slope_distance;
       slope_final_velocity = (velocity_squared > 0) ? sqrt(velocity_squared) : 0.0f;
       
       std::cout << "Slope physics: initial_v=" << initial_velocity << ", accel=" << slope_acceleration 
                 << ", distance=" << slope_distance << ", final_v=" << slope_final_velocity << std::endl;
       
       // Add slope segment
       TrajectorySegment slope_segment;
       slope_segment.start_position = start_pos;
       slope_segment.end_position = slope_end_pos;
       slope_segment.platform = pair.slope_platform;
       slope_segment.distance = slope_distance;
       trajectory_segments.push_back(slope_segment);
       
       // Calculate trajectory along second platform (any steepness)
       float second_platform_acceleration = gravity * sin(pair.horizontal_platform->rotation * DEG2RAD);
       float second_platform_friction = mu_kinetic * gravity * cos(abs(pair.horizontal_platform->rotation * DEG2RAD));
       float net_deceleration = second_platform_friction - second_platform_acceleration;
       
       // Calculate stopping distance on second platform
       float stopping_distance;
       if (net_deceleration > 0) {
           stopping_distance = (slope_final_velocity * slope_final_velocity) / (2.0f * net_deceleration);
       } else {
           // Platform accelerates the box - won't stop on this platform
           stopping_distance = std::numeric_limits<float>::max();
       }
       
       // Calculate direction along second platform
       Vector2 second_platform_direction = Vector2Normalize(Vector2Subtract(
           pair.horizontal_platform->top_right, pair.horizontal_platform->top_left));
       
       // Calculate final position on second platform
       Vector2 second_platform_end = Vector2Add(slope_end_pos, Vector2Scale(second_platform_direction, stopping_distance));
       
       // Make sure we don't go past the end of the second platform
       Vector2 platform_right_end = GetRightEndOfPlatform(*pair.horizontal_platform);
       float max_platform_distance = Vector2Distance(slope_end_pos, platform_right_end);
       
       if (stopping_distance > max_platform_distance) {
           // Box doesn't stop on this platform - continues with remaining velocity
           second_platform_end = platform_right_end;
           
           // Calculate remaining velocity after traveling the platform length
           float distance_traveled = max_platform_distance;
           if (net_deceleration > 0) {
               float velocity_lost_squared = 2.0f * net_deceleration * distance_traveled;
               float remaining_velocity_squared = slope_final_velocity * slope_final_velocity - velocity_lost_squared;
               final_velocity = (remaining_velocity_squared > 0) ? sqrt(remaining_velocity_squared) : 0.0f;
           } else {
               // Platform accelerates - calculate final velocity
               float velocity_gained_squared = 2.0f * abs(net_deceleration) * distance_traveled;
               final_velocity = sqrt(slope_final_velocity * slope_final_velocity + velocity_gained_squared);
           }
           
           std::cout << "Box continues past second platform with velocity: " << final_velocity << std::endl;
       } else {
           // Box stops on second platform
           final_velocity = 0.0f;
           std::cout << "Box stops on second platform" << std::endl;
       }
       
       // Add second platform segment
       TrajectorySegment second_segment;
       second_segment.start_position = slope_end_pos;
       second_segment.end_position = second_platform_end;
       second_segment.platform = pair.horizontal_platform;
       second_segment.distance = Vector2Distance(slope_end_pos, second_platform_end);
       trajectory_segments.push_back(second_segment);
       
       end_pos = second_platform_end;
       return true;
   }
   // Case 2: Single platform reference frame
   else {
       std::cout << "Calculating single-platform reference frame" << std::endl;
       
       Vector2 platform_end_pos = GetRightEndOfPlatform(*pair.slope_platform);
       float platform_distance = GetDistanceAlongSlope(start_pos, platform_end_pos, *pair.slope_platform);
       
       // Calculate final velocity after platform (any steepness)
       float platform_acceleration = gravity * sin(pair.slope_platform->rotation * DEG2RAD);
       float platform_friction = mu_kinetic * gravity * cos(abs(pair.slope_platform->rotation * DEG2RAD));
       float net_acceleration = platform_acceleration - platform_friction;
       
       float velocity_squared = initial_velocity * initial_velocity + 2.0f * net_acceleration * platform_distance;
       final_velocity = (velocity_squared > 0) ? sqrt(velocity_squared) : 0.0f;
       
       // Add platform segment
       TrajectorySegment platform_segment;
       platform_segment.start_position = start_pos;
       platform_segment.end_position = platform_end_pos;
       platform_segment.platform = pair.slope_platform;
       platform_segment.distance = platform_distance;
       trajectory_segments.push_back(platform_segment);
       
       end_pos = platform_end_pos;
       return true;
   }
}

bool Box::CanTransitionToNextPlatform(Vector2 position, float velocity, const Platform* current_platform, const Platform* next_platform) {
   if (!next_platform) return false;
   
   // Original teleport behavior - no velocity restrictions
   // The teleport system handles any slope steepness automatically
   // Future: This is where jump mechanics can be added if needed
   
   return true; // Allow transition to any platform regardless of steepness
}