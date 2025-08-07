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
                if (velocity.x > 0.1f && is_colliding && current_platform_id >= 0) {
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
                    closest_platform_id = i;
                    closest_point = closestPointOnPlatform;
                }
            }
        }
    }
    
    if (closest_platform_id >= 0) {
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
    
    // NEW: Check if this is an upward ramp (negative rotation means upward slope)
    bool is_upward_ramp = current_platform.rotation < -5.0f; // Threshold for upward slopes
    
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
        // NEW: For upward ramps, allow the box to launch off instead of teleporting
        if (is_upward_ramp) {
            // Check if box has enough upward velocity to launch
            if (velocity.y < -1.0f) { // Negative y velocity means moving upward
                // Let the box launch - stop collision detection
                is_colliding = false;
                current_platform_id = -1;
                // Don't reset prediction start - let it continue in projectile motion
                return true; // Transition handled by launching
            }
            // If not enough upward velocity, box will just slide off the end
            // Fall through to normal transition logic
        }
        
        // EXISTING LOGIC for downward/horizontal platforms
        const Platform* next_platform = nullptr;
        
        // Get the right end of current platform
        Vector2 current_right_end_pos = GetRightEndOfPlatform(current_platform);
        
        // Search for a platform that starts near the current platform's right end
        float best_distance = std::numeric_limits<float>::max();
        
        for (size_t i = 0; i < platforms.size(); i++) {
            if ((int)i == current_platform_id) continue;
            
            const Platform& candidate = platforms[i];
            
            // Get the left end of candidate platform
            Vector2 candidate_left = (candidate.top_left.x < candidate.top_right.x) ? 
                                   candidate.top_left : candidate.top_right;
            
            // Check if this candidate is to the right and close to current platform's end
            float distance = Vector2Distance(current_right_end_pos, candidate_left);
            
            // Also check if they intersect (platforms meet)
            Vector2 intersection;
            bool intersects = CheckCollisionLines(
                current_platform.top_left, current_platform.top_right,
                candidate.top_left, candidate.top_right,
                &intersection
            );
            
            // Accept platform if it's close enough or intersects
            if ((distance < 50.0f || intersects) && distance < best_distance) {
                best_distance = distance;
                next_platform = &candidate;
            }
        }
        
        // NEW: If no next platform found and this is an upward ramp, launch the box
        if (!next_platform && is_upward_ramp) {
            is_colliding = false;
            current_platform_id = -1;
            return true; // Let box continue in projectile motion
        }
        
        if (!next_platform) {
            return false; // No suitable next platform found for regular platforms
        }
        
        // EXISTING transition logic for connected platforms
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
                
                // Preserve speed, redirect along new platform
                velocity = Vector2Scale(next_slope_direction, current_speed);
                rotation = next_platform->rotation;
                
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
    
    // Draw all trajectory segments
    for (size_t i = 0; i < trajectory_segments.size(); i++) {
        const auto& segment = trajectory_segments[i];
        
        if (segment.platform == nullptr) {
            // This is a projectile motion segment - draw the curved path
            if (!projectile_trajectory_points.empty()) {
                for (size_t j = 1; j < projectile_trajectory_points.size(); j++) {
                    DrawLineEx(projectile_trajectory_points[j-1], projectile_trajectory_points[j], 3.0f, MAGENTA);
                }
                // Draw projectile path points
                for (const auto& point : projectile_trajectory_points) {
                    DrawCircleV(point, 2.0f, PINK);
                }
            }
        } else {
            // Normal platform segment
            Color line_color = (abs(segment.platform->rotation) > 5.0f) ? ORANGE : YELLOW;
            DrawLineEx(segment.start_position, segment.end_position, 5.0f, line_color);
        }
        
        // Draw transition points
        DrawCircleV(segment.end_position, 5, GREEN);
    }
    
    // Draw starting position
    DrawCircleV(prediction_start_position, 4, PURPLE);
    
    // Draw final ghost box
    if (!trajectory_segments.empty()) {
        float border = 2.0f;
        DrawRectanglePro(
            (Rectangle){ final_ghost_position.x + size.x * 0.5f, final_ghost_position.y - size.y * 0.5f, size.x, size.y }, 
            (Vector2){ size.x * 0.5f, size.y * 0.5f }, 
            0.0f,
            (Color){255, 255, 255, 100}
        );
        
        DrawRectanglePro(
            (Rectangle){ final_ghost_position.x + size.x * 0.5f, final_ghost_position.y - size.y * 0.5f, size.x - border * 2, size.y - border * 2 }, 
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
    float tolerance = 100.0f; // Increased tolerance significantly
    const Platform* best_platform = nullptr;
    float min_distance = std::numeric_limits<float>::max();
    Vector2 current_right_end = GetRightEndOfPlatform(current_platform);
    
    // Finding next platform
    
    for (size_t i = 0; i < platforms.size(); i++) {
        const auto& platform = platforms[i];
        if (&platform == &current_platform) continue;
        
        // Check for direct intersection first
        Vector2 intersection;
        if (CheckCollisionLines(current_platform.top_left, current_platform.top_right,
                               platform.top_left, platform.top_right, &intersection)) {
            // Found intersecting platform
            return &platform;
        }
        
        // Get platform endpoints
        Vector2 platform_left_end = (platform.top_left.x < platform.top_right.x) ? platform.top_left : platform.top_right;
        Vector2 platform_right_end = (platform.top_left.x > platform.top_right.x) ? platform.top_left : platform.top_right;
        
        // CRITICAL: Only consider platforms that are ACTUALLY to the right
        // A platform is "to the right" if its LEFT end is to the right of current platform's RIGHT end
        if (platform_left_end.x <= current_right_end.x) {
            // Platform too far left
            continue; // This platform is to the left or overlapping
        }
        
        // Calculate distance to this rightward platform
        float dist_to_left = Vector2Distance(current_right_end, platform_left_end);
        float dist_to_right = Vector2Distance(current_right_end, platform_right_end);
        float min_dist = std::min(dist_to_left, dist_to_right);
        
        // Platform evaluation
        
        if (min_dist < tolerance && min_dist < min_distance) {
            min_distance = min_dist;
            best_platform = &platform;
            // New best platform found
        }
    }
    
    if (best_platform) {
        // Selected platform
    } else {
        // No platform found
    }
    
    return best_platform;
}

// ================================
// NEW REFERENCE FRAME-BASED TRAJECTORY CALCULATION
// ================================

void Box::CalculateMultiReferenceFrameTrajectory(const std::vector<Platform>& platforms) {
    trajectory_segments.clear();
    
    // POSITION FIX: Start from bottom right corner to match collision detection
    float box_rotation_rad = rotation * DEG2RAD;
    float cos_rot = cosf(box_rotation_rad);
    float sin_rot = sinf(box_rotation_rad);
    
    // Local coordinates of right bottom corner (same as collision detection)
    Vector2 local_right_bottom = { size.x * 0.5f, size.y * 0.5f };
    
    // Rotate and translate to world coordinates
    Vector2 current_start_position = {
        local_right_bottom.x * cos_rot - local_right_bottom.y * sin_rot + position.x,
        local_right_bottom.x * sin_rot + local_right_bottom.y * cos_rot + position.y
    };
    
    // CRITICAL FIX: Use speed along slope direction, not total speed magnitude
    // This matches the real physics which projects velocity onto slope
    float current_speed = 0.0f;  // Will be calculated per platform
    
    // Find starting platform
    const Platform* current_slope_platform = nullptr;
    for (const auto& platform : platforms) {
        if (IsPointOnPlatform(current_start_position, platform)) {
            current_slope_platform = &platform;
            break;
        }
    }
    
    if (!current_slope_platform) {
        return;
    }
    
    // FIXED: Create proper platform sequence by sorting by position
    std::vector<const Platform*> sorted_platforms;
    for (const auto& platform : platforms) {
        sorted_platforms.push_back(&platform);
    }
    
    std::sort(sorted_platforms.begin(), sorted_platforms.end(), 
        [](const Platform* a, const Platform* b) {
            float a_left = std::min(a->top_left.x, a->top_right.x);
            float b_left = std::min(b->top_left.x, b->top_right.x);
            return a_left < b_left;
        });
    
    // CRITICAL FIX: Find which reference frame the box is actually on
    int starting_frame = -1;
    for (int frame = 0; frame < 2 && frame * 2 + 1 < (int)sorted_platforms.size(); frame++) {
        const Platform* slope_platform = sorted_platforms[frame * 2];
        const Platform* horizontal_platform = sorted_platforms[frame * 2 + 1];
        
        // Check if box is on either platform in this reference frame
        if (current_slope_platform == slope_platform || current_slope_platform == horizontal_platform) {
            starting_frame = frame;
            break;
        }
    }
    
    if (starting_frame == -1) {
        return; // Box not on any known reference frame
    }
    
    // Process reference frames starting from where the box actually is
    int max_frames = std::min(2 - starting_frame, (int)sorted_platforms.size() / 2 - starting_frame);
    
    for (int frame_offset = 0; frame_offset < max_frames; frame_offset++) {
        int frame = starting_frame + frame_offset;
        
        // Simple pairing: slope at even index, horizontal at odd index  
        const Platform* slope_platform = sorted_platforms[frame * 2];
        const Platform* horizontal_platform = (frame * 2 + 1 < (int)sorted_platforms.size()) ? 
                                            sorted_platforms[frame * 2 + 1] : nullptr;
        
        if (!slope_platform) {
            final_ghost_position = current_start_position;
            break;
        }
        
        // EXACT SAME CALCULATION AS ORIGINAL WORKING CODE  
        Vector2 intersect;
        bool found_intersection = false;
        Vector2 transition_point;
        
        if (horizontal_platform) {
            found_intersection = CheckCollisionLines(slope_platform->top_left, slope_platform->top_right, 
                                   horizontal_platform->top_left, horizontal_platform->top_right, &intersect);
        }
        
        if (found_intersection) {
            transition_point = intersect;
        } else {
            // Use right end of slope as launch point
            transition_point = GetRightEndOfPlatform(*slope_platform);
        }
        
        // CRITICAL FIX: Calculate initial speed along THIS slope direction (matches real physics)
        if (frame_offset == 0) {
            // For first frame we're actually calculating, project current velocity onto slope direction
            float slope_angle = slope_platform->rotation * DEG2RAD;
            Vector2 slope_direction = { cos(slope_angle), sin(slope_angle) };
            current_speed = Vector2DotProduct(velocity, slope_direction);
            // Ensure positive speed (moving in slope direction)
            current_speed = fabs(current_speed);
            
            // ADDITIONAL FIX: If box is on horizontal platform, start from horizontal calculations
            if (current_slope_platform == horizontal_platform) {
                // Box is on horizontal platform - calculate remaining distance on this platform
                Vector2 platform_right_end = GetRightEndOfPlatform(*horizontal_platform);
                float remaining_distance = Vector2Distance(current_start_position, platform_right_end);
                
                // Use current speed as horizontal speed
                current_speed = Vector2Length(velocity);
                
                // Skip slope calculation, go directly to horizontal segment
                Vector2 horizontal_direction = Vector2Normalize(Vector2Subtract(horizontal_platform->top_right, horizontal_platform->top_left));
                Vector2 horizontal_end_position = Vector2Add(current_start_position, Vector2Scale(horizontal_direction, remaining_distance));
                
                // Add horizontal segment  
                TrajectorySegment horizontal_segment;
                horizontal_segment.start_position = current_start_position;
                horizontal_segment.end_position = horizontal_end_position;
                horizontal_segment.platform = horizontal_platform;
                horizontal_segment.distance = remaining_distance;
                trajectory_segments.push_back(horizontal_segment);
                
                final_ghost_position = horizontal_end_position;
                break; // For now, just show remaining distance on current platform
            }
        }
        
        // Calculate slope segment (same as original working code)
        float slope_distance = GetDistanceAlongSlope(current_start_position, transition_point, *slope_platform);
        float slope_final_velocity = CalculateFinalVelocity(current_speed, *slope_platform, slope_distance);
        
        // Add slope segment
        TrajectorySegment slope_segment;
        slope_segment.start_position = current_start_position;
        slope_segment.end_position = transition_point;
        slope_segment.platform = slope_platform;
        slope_segment.distance = slope_distance;
        trajectory_segments.push_back(slope_segment);
        
        // NEW: Check if this is a jump scenario
        bool is_jump = false;
        Vector2 launch_velocity = {0, 0};
        
        if (!horizontal_platform || !found_intersection) {
            // No next platform or no intersection = jump scenario
            is_jump = true;
            
            // Calculate launch velocity from slope
            float slope_angle = slope_platform->rotation * DEG2RAD;
            Vector2 slope_direction = { cos(slope_angle), sin(slope_angle) };
            launch_velocity = Vector2Scale(slope_direction, slope_final_velocity);
        }
        
        if (is_jump) {
            // Calculate projectile motion trajectory
            Vector2 projectile_segments = CalculateProjectileTrajectory(transition_point, launch_velocity, platforms);
            final_ghost_position = projectile_segments;
            break; // End trajectory calculation after jump
        }
        
        // Continue with existing horizontal platform logic if not jumping
        if (horizontal_platform) {
            float slope_angle_horiz = horizontal_platform->rotation * DEG2RAD;
            
            // FRICTION FIX: Use same friction calculation as real physics
            float horizontal_deceleration = mu_kinetic * gravity * cos(fabs(slope_angle_horiz));
            
            // Add slope component if platform isn't perfectly horizontal
            float gravity_component = gravity * sin(slope_angle_horiz);
            float net_deceleration = horizontal_deceleration - gravity_component;
            
            // Prevent negative deceleration (box would accelerate)
            if (net_deceleration <= 0) {
                // Box will accelerate/maintain speed - goes to end of platform
                net_deceleration = 0.001f; // Small value to prevent division by zero
            }
            
            float horizontal_stopping_distance = (slope_final_velocity * slope_final_velocity) / (2.0f * net_deceleration);
            
            Vector2 horizontal_direction = Vector2Normalize(Vector2Subtract(horizontal_platform->top_right, horizontal_platform->top_left));
            Vector2 horizontal_end_position = Vector2Add(transition_point, Vector2Scale(horizontal_direction, horizontal_stopping_distance));
            
            // Check if we go past the end of horizontal platform
            Vector2 platform_right_end = GetRightEndOfPlatform(*horizontal_platform);
            float max_horizontal_distance = Vector2Distance(transition_point, platform_right_end);
            float actual_horizontal_distance = horizontal_stopping_distance;
            float remaining_velocity = 0.0f;
            
            if (horizontal_stopping_distance > max_horizontal_distance) {
                // Box doesn't stop on this platform - calculate remaining velocity
                horizontal_end_position = platform_right_end;
                actual_horizontal_distance = max_horizontal_distance;
                
                // Energy conservation: v² = u² - 2as (deceleration removes energy)
                float velocity_lost_squared = 2.0f * net_deceleration * actual_horizontal_distance;
                float remaining_velocity_squared = slope_final_velocity * slope_final_velocity - velocity_lost_squared;
                remaining_velocity = (remaining_velocity_squared > 0) ? sqrt(remaining_velocity_squared) : 0.0f;
                
                // PHYSICS FIX: Match real friction behavior - stop if very low velocity
                if (remaining_velocity < 0.5f) {
                    remaining_velocity = 0.0f;
                }
                
                // Box continues to next platform
            } else {
                // Box stops on this platform
                remaining_velocity = 0.0f;
            }
            
            // Add horizontal segment
            TrajectorySegment horizontal_segment;
            horizontal_segment.start_position = transition_point;
            horizontal_segment.end_position = horizontal_end_position;
            horizontal_segment.platform = horizontal_platform;
            horizontal_segment.distance = actual_horizontal_distance;
            trajectory_segments.push_back(horizontal_segment);
            
            // Stop if no remaining velocity
            if (remaining_velocity < 0.1f) {
                final_ghost_position = horizontal_end_position;
                break;
            }
            
            // Set up for next reference frame
            current_speed = remaining_velocity;
            current_start_position = horizontal_end_position; // Continue from end of horizontal platform
        }
    }
    
    // Final result - minimal output
    multi_platform_ghost_calculated = !trajectory_segments.empty();
}

// NEW: Calculate projectile motion trajectory
Vector2 Box::CalculateProjectileTrajectory(Vector2 launch_position, Vector2 launch_velocity, const std::vector<Platform>& platforms) {
    // Projectile motion parameters
    float dt = 0.02f; // Smaller time step for more accurate trajectory
    Vector2 current_pos = launch_position;
    Vector2 current_vel = launch_velocity;
    float max_time = 10.0f; // Maximum simulation time to prevent infinite loops
    float time = 0.0f;
    
    // Store projectile trajectory points for drawing
    std::vector<Vector2> projectile_path;
    projectile_path.push_back(current_pos);
    
    while (time < max_time) {
        // Store previous position for collision checking
        Vector2 prev_pos = current_pos;
        
        // Update velocity first (gravity affects velocity)
        current_vel.y += gravity * dt;
        
        // Then update position using current velocity
        current_pos = Vector2Add(current_pos, Vector2Scale(current_vel, dt));
        
        // Add point to trajectory path every few iterations for smoother drawing
        if ((int)(time / dt) % 3 == 0) { // Every 3rd point
            projectile_path.push_back(current_pos);
        }
        
        // Check for collision with any platform
        for (const auto& platform : platforms) {
            // Check if trajectory crosses platform top surface
            Vector2 intersection;
            bool crosses_platform = CheckCollisionLines(prev_pos, current_pos, platform.top_left, platform.top_right, &intersection);
            
            if (crosses_platform) {
                // Make sure we're hitting the platform from above
                if (prev_pos.y <= intersection.y && current_vel.y > 0) {
                    // Add final point to trajectory
                    projectile_path.push_back(intersection);
                    
                    // Add projectile segment to trajectory
                    TrajectorySegment projectile_segment;
                    projectile_segment.start_position = launch_position;
                    projectile_segment.end_position = intersection;
                    projectile_segment.platform = nullptr; // No platform for projectile motion
                    projectile_segment.distance = Vector2Distance(launch_position, intersection);
                    trajectory_segments.push_back(projectile_segment);
                    
                    // Store projectile path for drawing
                    projectile_trajectory_points = projectile_path;
                    
                    // CONTINUE SIMULATION: Calculate movement along the landing platform
                    Vector2 final_position = CalculatePostLandingTrajectory(intersection, current_vel, platform);
                    
                    return final_position;
                }
            }
        }
        
        // Check if projectile hits ground (screen bottom)
        if (current_pos.y >= GetScreenHeight() - 10) { // Small buffer from bottom
            Vector2 ground_collision = { current_pos.x, (float)GetScreenHeight() - 10 };
            projectile_path.push_back(ground_collision);
            
            // Add projectile segment
            TrajectorySegment projectile_segment;
            projectile_segment.start_position = launch_position;
            projectile_segment.end_position = ground_collision;
            projectile_segment.platform = nullptr;
            projectile_segment.distance = Vector2Distance(launch_position, ground_collision);
            trajectory_segments.push_back(projectile_segment);
            
            // Store projectile path for drawing
            projectile_trajectory_points = projectile_path;
            
            return ground_collision;
        }
        
        // Check if projectile goes off screen horizontally
        if (current_pos.x < 0 || current_pos.x > GetScreenWidth()) {
            projectile_path.push_back(current_pos);
            
            TrajectorySegment projectile_segment;
            projectile_segment.start_position = launch_position;
            projectile_segment.end_position = current_pos;
            projectile_segment.platform = nullptr;
            projectile_segment.distance = Vector2Distance(launch_position, current_pos);
            trajectory_segments.push_back(projectile_segment);
            
            projectile_trajectory_points = projectile_path;
            return current_pos;
        }
        
        time += dt;
    }
    
    // If no collision found, return the final position
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