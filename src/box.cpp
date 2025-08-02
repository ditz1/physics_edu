#include <box.hpp>
#include "collision_utils.hpp"
#include <iostream>
#include <algorithm>
#include <limits>

Box::Box() {
    size = {50.0f, 50.0f};
}
Box::Box(int w, int h) {
    size = {static_cast<float>(w), static_cast<float>(h)};
}

Box::~Box() {

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
    // Draw with DrawRectangleV (should work)
    //DrawRectangleV(position, size, RED);
    float border = 2.0f;
    // Draw with DrawRectanglePro - see where it appears
    
    DrawRectanglePro(
        (Rectangle){ position.x, position.y, size.x, size.y }, 
        (Vector2){ size.x * 0.5f, size.y * 0.5f }, // origin at center
        rotation, 
        BLACK
    );
    
    DrawRectanglePro(
        (Rectangle){ position.x, position.y, size.x - border * 2, size.y - border * 2 }, 
        (Vector2){ (size.x - border * 2) * 0.5f, (size.y - border * 2) * 0.5f }, // origin at center of smaller rect
        rotation, 
        color
    );

    //DrawTextureEx(*texture, { position.x - size.x, position.y - size.y }, rotation, 0.08f, WHITE);
    
    
    DrawCircleV(position, 5, WHITE);
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
    
    // Get the two vertical lines from the box's bottom corners
    std::vector<LineSegment> lines = CollisionUtils::GetBoxBottomLines(position, size, rotation);
    
    // Check collision with all platforms using the two-line system
    TwoLineCollisionInfo collisionInfo = CollisionUtils::CheckTwoLineCollision(lines, platforms);
    
    if (collisionInfo.hasCollision) {
        bool isNewCollision = !was_colliding_last_frame;
        bool isPlatformTransition = (was_colliding_last_frame && last_platform_id != collisionInfo.platformId);
        
        // Store old velocity for platform transitions
        Vector2 old_velocity = velocity;
        float old_speed = Vector2Length(velocity);
        
        is_colliding = true;
        current_platform_id = collisionInfo.platformId;
        int old_rotation = rotation;
        
        // Get the platform we're colliding with
        const Platform& platform = platforms[collisionInfo.platformId];
        rotation = platform.rotation;
        
        // Simple positioning: place box on platform surface when collision is detected
        if (isNewCollision || collisionInfo.distanceToPlatform < 0.2f) {
            // Calculate the box's bottom center point
            Vector2 boxBottomCenter = {
                position.x,
                position.y + size.y * 0.5f
            };
            
            // Find the closest point on the platform surface to the box bottom center
            Vector2 platformDir = Vector2Subtract(platform.top_right, platform.top_left);
            Vector2 toBox = Vector2Subtract(boxBottomCenter, platform.top_left);
            float platformLength = Vector2Length(platformDir);
            
            if (platformLength > 0) {
                Vector2 normalizedDir = Vector2Normalize(platformDir);
                float t = Vector2DotProduct(toBox, normalizedDir) / platformLength;
                t = fmax(0.0f, fmin(1.0f, t)); // Clamp to platform bounds
                
                // Calculate the closest point on the platform surface
                Vector2 closestPoint = Vector2Add(platform.top_left, Vector2Scale(normalizedDir, t * platformLength));
                
                // Calculate the distance from box bottom to platform surface
                float distanceToMove = boxBottomCenter.y - closestPoint.y;
                
                if (distanceToMove > 0) {
                    position.y -= distanceToMove;
                }
            }
        }
        
        // Handle velocity - simple platform following
        if (isPlatformTransition) {
            // Simple transition: just follow the new platform direction
            float new_slope_angle = platform.rotation * DEG2RAD;
            Vector2 new_slope_direction = {cos(new_slope_angle), sin(new_slope_angle)};
            
            // Keep the same speed, just redirect along the new platform
            velocity = Vector2Scale(new_slope_direction, old_speed);
        } else if (isNewCollision) {
            // For new collisions, just remove the downward velocity component
            float velocityDotNormal = Vector2DotProduct(velocity, collisionInfo.normal);
            if (velocityDotNormal < 0) {
                Vector2 velocityAlongNormal = Vector2Scale(collisionInfo.normal, velocityDotNormal);
                velocity = Vector2Subtract(velocity, velocityAlongNormal);
            }
            
            // Set prediction start position and calculate ghost trajectory when box first lands on a platform
            // Only do this for new collisions, not platform transitions, and only if we haven't already calculated it
            if (isNewCollision && !has_prediction_start) {
                SetPredictionStartPosition();
                CalculateGhostTrajectory(platforms);
            }
        }
    } else {
        // Only set is_colliding to false if we were actually colliding before
        // This prevents constant state flipping on horizontal platforms
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
    
    // Threshold for transition detection - when the right corner gets very close
    float transition_threshold = 10.0f; // Small threshold for accurate transition
    
    if (distance_to_right_end < transition_threshold && velocity.x > 0) {
        // Find the next platform to transition to
        const Platform* next_platform = FindNextPlatformToRight(current_platform, platforms);
        
        if (next_platform) {
            Vector2 transition_point;
            bool has_intersection = CheckCollisionLines(
                current_platform.top_left, current_platform.top_right,
                next_platform->top_left, next_platform->top_right, 
                &transition_point
            );
            
            if (has_intersection) {
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
                
                // Preserve speed, redirect along new platform - NO energy loss
                velocity = Vector2Scale(next_slope_direction, current_speed);
                rotation = next_platform->rotation;
                
                std::cout << "TRANSITION TELEPORT! From platform " << last_platform_id 
                          << " to next platform. Speed preserved: " << current_speed << std::endl;
                
                return true; // Transition handled
            }
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
        CalculateMultiPlatformTrajectory(platforms);
        multi_platform_ghost_calculated = true;
    }
    
    // Draw all trajectory segments
    for (size_t i = 0; i < trajectory_segments.size(); i++) {
        const auto& segment = trajectory_segments[i];
        
        // Choose color based on platform type
        Color line_color = (abs(segment.platform->rotation) > 5.0f) ? ORANGE : YELLOW;
        
        // Draw trajectory line
        DrawLineEx(segment.start_position, segment.end_position, 3.0f, line_color);
        
        // Draw platform top line for reference
        Color platform_color = (abs(segment.platform->rotation) > 5.0f) ? RED : BLUE;
        DrawLineEx(segment.platform->top_left, segment.platform->top_right, 2.0f, platform_color);
        
        // Draw transition points
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
    }
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
    float tolerance = 50.0f; // Increased tolerance for better platform finding
    const Platform* best_platform = nullptr;
    float min_distance = std::numeric_limits<float>::max();
    Vector2 current_right_end = GetRightEndOfPlatform(current_platform);
    
    for (const auto& platform : platforms) {
        if (&platform == &current_platform) continue;
        
        // Check for direct intersection first
        Vector2 intersection;
        if (CheckCollisionLines(current_platform.top_left, current_platform.top_right,
                               platform.top_left, platform.top_right, &intersection)) {
            // Direct intersection found - this is definitely connected
            return &platform;
        }
        
        // Only consider platforms that are to the right and close
        Vector2 platform_left_end = (platform.top_left.x < platform.top_right.x) ? platform.top_left : platform.top_right;
        Vector2 platform_right_end = (platform.top_left.x > platform.top_right.x) ? platform.top_left : platform.top_right;
        
        // Check if this platform is to the right of current platform
        if (platform_left_end.x < current_right_end.x - tolerance) {
            continue; // This platform is too far left
        }
        
        // Check proximity to the right end of current platform
        float dist_to_left = Vector2Distance(current_right_end, platform_left_end);
        float dist_to_right = Vector2Distance(current_right_end, platform_right_end);
        float min_dist = std::min(dist_to_left, dist_to_right);
        
        if (min_dist < tolerance && min_dist < min_distance) {
            min_distance = min_dist;
            best_platform = &platform;
        }
    }
    
    return best_platform;
}