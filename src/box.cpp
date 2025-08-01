#include <box.hpp>
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

void Box::Update(float dt) {
    // p1 = p0 + v * dt
    // if box was not colliding last frame but it is colliding this frame, we need to reset the velocity

    if (!was_colliding_last_frame && is_colliding) {
        //velocity = {0.0f, 0.0f};
        acceleration.y = 9.81f;
    }

    if (!is_colliding) {
        ApplyGravity(dt);
    } else {
        float slope_angle = rotation * DEG2RAD;
        float gravity_along_slope = gravity * sin(slope_angle);
        
        // Calculate slope direction (assuming rotation is the platform angle)
        Vector2 slope_direction = { cos(slope_angle), sin(slope_angle) };
        
        // Apply acceleration along slope
        Vector2 slope_accel = Vector2Scale(slope_direction, gravity_along_slope);
        velocity = Vector2Add(velocity, Vector2Scale(slope_accel, dt));
    }
    ApplyFriction(dt);

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
    
    // DrawRectanglePro(
    //     (Rectangle){ position.x, position.y, size.x - border * 2, size.y - border * 2 }, 
    //     (Vector2){ (size.x - border * 2) * 0.5f, (size.y - border * 2) * 0.5f }, // origin at center of smaller rect
    //     rotation, 
    //     color
    // );

    DrawTextureEx(*texture, { position.x - size.x, position.y - size.y }, rotation, 0.08f, WHITE);
    
    
    // Draw a circle at the position for reference
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

    // if (velocity.y < 0.1f && velocity.y > -0.1f) {
    //     velocity.y = 0.0f; 
    //     acceleration.y = 0.0f; 
    // }
    // if (velocity.x < 0.1f && velocity.x > -0.1f) {
    //     velocity.x = 0.0f; 
    //     acceleration.x = 0.0f; 
    // }
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

void Box::CheckPlatformCollisionSAT(const Platform& platform, int platform_id) {
    RotatedRectangle boxRect = {
        { position.x , position.y },
        size,
        0.0f
    };
    
    RotatedRectangle platformRect = {
        platform.position,
        platform.size,
        platform.rotation * DEG2RAD
    };
    
    CollisionInfo collision = CollisionUtils::GetCollisionInfo(boxRect, platformRect);
    
    if (collision.hasCollision) {
        bool isNewCollision = !was_colliding_last_frame;
        bool isPlatformTransition = (was_colliding_last_frame && last_platform_id != platform_id);
        
        // Store old velocity for platform transitions
        Vector2 old_velocity = velocity;
        float old_speed = Vector2Length(velocity);
        
        is_colliding = true;
        current_platform_id = platform_id;
        int old_rotation = rotation; // Store old rotation for debugging
        rotation = platform.rotation;
        
        // Move box out of collision
        Vector2 correction = Vector2Scale(collision.normal, collision.penetration);
        position = Vector2Subtract(position, correction);
        
        // Handle velocity during platform transitions
        if (isPlatformTransition) {
            // Convert velocity to the new platform's coordinate system
            float new_slope_angle = platform.rotation * DEG2RAD;
            Vector2 new_slope_direction = {cos(new_slope_angle), sin(new_slope_angle)};
            
            // Project the old velocity onto the new slope direction
            float velocity_along_new_slope = Vector2DotProduct(old_velocity, new_slope_direction);
            std::cout << "old rotation: " << old_rotation << " | new rotation: " << rotation << std::endl;
            
            // If we're transitioning to a flatter surface, convert some y-velocity to x-velocity
            if (abs(rotation) < abs(old_rotation)) { // New platform is flatter
                // Preserve most of the speed but redirect it along the new surface
                // again, issue here is that this is just completely arbitrary. i dont know if the conservation would be 10%,
                // or anywhere close for that matter, but otherwise the box wont move at all after transition
                std::cout << "transitioning to flatter surface" << std::endl;
                velocity = Vector2Scale(new_slope_direction, old_speed * 1.0f); // 20%
            } else {
                // Normal velocity projection for other transitions
                velocity = Vector2Scale(new_slope_direction, velocity_along_new_slope);
            }
        } else if (isNewCollision) {
            // Only remove velocity component for brand NEW collisions
            float velocityDotNormal = Vector2DotProduct(velocity, collision.normal);
            if (velocityDotNormal < 0) {
                Vector2 velocityAlongNormal = Vector2Scale(collision.normal, velocityDotNormal);
                velocity = Vector2Subtract(velocity, velocityAlongNormal);
            }
        }
        
        //std::cout << "transition from platform " << last_platform_id << " to " << platform_id << " | " << velocity.x << std::endl;
    }
}

void Box::ApplyFriction(float dt) {
    if (is_colliding) {
        float slope_angle = rotation * DEG2RAD;
        float normal_force = mass * gravity * cos(slope_angle);
        
        // Get the slope tangent direction (direction along the slope)
        Vector2 slope_tangent = {cos(slope_angle), sin(slope_angle)};
        
        // Project velocity onto the slope direction
        float velocity_along_slope = Vector2DotProduct(velocity, slope_tangent);
        
        if (abs(velocity_along_slope) > 0.001f) {
            float friction_force = mu_kinetic * normal_force;
            float friction_accel = friction_force / mass;
            
            // Apply friction opposite to the direction of motion along slope
            Vector2 friction_vector;
            if (velocity_along_slope > 0) {
                friction_vector = Vector2Scale(slope_tangent, -friction_accel);
            } else {
                friction_vector = Vector2Scale(slope_tangent, friction_accel);
            }
            
            // Apply friction, but don't let it reverse the velocity direction
            Vector2 friction_impulse = Vector2Scale(friction_vector, dt);
            if (abs(velocity_along_slope) > abs(Vector2DotProduct(friction_impulse, slope_tangent))) {
                velocity = Vector2Add(velocity, friction_impulse);
            } else {
                // Stop motion along slope if friction would reverse it
                Vector2 velocity_along_slope_vec = Vector2Scale(slope_tangent, velocity_along_slope);
                velocity = Vector2Subtract(velocity, velocity_along_slope_vec);
            }
        }
    }
}


float Box::CalculateStoppingDistanceFromSlope(const Platform& slope_platform, float slope_travel_distance) {
    //
    // on slope
    //
    
    float slope_angle = slope_platform.rotation * DEG2RAD;
    
    // V_initial = 0 (starts from rest)
    float V_initial_slope = 0.0f;
    
    // acceleration down the slope
    float gravity_component = gravity * sin(fabs(slope_angle));
    float friction_component_slope = mu_kinetic * gravity * cos(fabs(slope_angle));
    float a_slope = gravity_component - friction_component_slope;
    
    if (a_slope <= 0) {
        return 0.0f; // Won't slide down slope
    }
    
    // V_final^2 = V_initial^2 + 2*a*d
    // V_final^2 = 0^2 + 2 * a_slope * slope_travel_distance
    float V_final_slope_squared = 2.0f * a_slope * slope_travel_distance;
    float V_final_slope = sqrt(V_final_slope_squared);

    //
    // horizontal platform
    //

    // V_initial = V_final_slope (velocity gained from slope)
    float V_initial_horizontal = V_final_slope;
    
    // V_final = 0 (comes to rest)
    float V_final_horizontal = 0.0f;
    
    // Deceleration due to friction on horizontal surface
    float a_horizontal = -mu_kinetic * gravity;
    
    // V_final^2 = V_initial^2 + 2*a*d
    // 0^2 = V_initial_horizontal^2 + 2*a_horizontal*d
    // d = -V_initial_horizontal^2 / (2*a_horizontal)
    float stopping_distance = -(V_initial_horizontal * V_initial_horizontal) / (2.0f * a_horizontal);
    
    return stopping_distance;
}

float Box::GetSlopeDistance(const Platform& platform) {
    return platform.size.x; // doesnt actually work - just calculate by need
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
    // Calculate the bottom center of the rotated box
    float box_angle = rotation * DEG2RAD;
    float cos_rot = cosf(box_angle);
    float sin_rot = sinf(box_angle);
    
    // Local bottom center point (relative to box center)
    Vector2 local_bottom_center = {-size.x * 0.5f, size.y * 0.5f};
    
    // Rotate and translate to world coordinates
    prediction_start_position = {
        local_bottom_center.x * cos_rot - local_bottom_center.y * sin_rot + position.x,
        local_bottom_center.x * sin_rot + local_bottom_center.y * cos_rot + position.y
    };
    
    has_prediction_start = true;
    multi_platform_ghost_calculated = false;
    ghost_calculated = false; // Reset ghost calculation
    std::cout << "Prediction start set at rotated bottom center: (" << prediction_start_position.x << ", " << prediction_start_position.y << ")" << std::endl;
}

void Box::DrawGhost(const Platform& slope_platform, const Platform& horizontal_platform) {
    if (!has_prediction_start) {
        return; // No prediction start position set yet
    }
    
    if (!ghost_calculated) {
        // Calculate once from the fixed starting position
        Vector2 slope_line_start = slope_platform.top_left;
        Vector2 slope_line_end = slope_platform.top_right;
        Vector2 hori_line_start = horizontal_platform.top_left;
        Vector2 hori_line_end = horizontal_platform.top_right;
        Vector2 intersect;

        if (CheckCollisionLines(slope_line_start, slope_line_end, hori_line_start, hori_line_end, &intersect)) {
            transition_point_stored = intersect;
            
            // Use FIXED starting position for calculation
            float slope_distance = Vector2Distance(prediction_start_position, transition_point_stored);
            float horizontal_stopping_distance = CalculateStoppingDistanceFromSlope(slope_platform, slope_distance);
            
            Vector2 horizontal_direction = {1.0f, 0.0f};
            if (slope_platform.rotation > 0) {
                horizontal_direction.x = 1.0f;
            } else {
                horizontal_direction.x = -1.0f;
            }
            
            ghost_position_stored = Vector2Add(transition_point_stored, 
                                             Vector2Scale(horizontal_direction, horizontal_stopping_distance));
            ghost_calculated = true;
            
            std::cout << "Ghost calculated - slope distance: " << slope_distance << ", horizontal distance: " << horizontal_stopping_distance << std::endl;
        } else {
            std::cout << "No intersection found between slope and horizontal platform." << std::endl;
            return;
        }
    }
    
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
    float slope_distance = Vector2Distance(prediction_start_position, transition_point_stored);
    DrawText(TextFormat("Start->Trans: %.1f", slope_distance), text_pos.x, text_pos.y, 14, ORANGE);
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
        
        // Calculate final velocity after traveling this distance
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

bool Box::IsPointOnPlatform(Vector2 point, const Platform& platform) {
    // Check if point is close to the platform's top surface
    float tolerance = 10.0f;
    
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
    
    // Acceleration components
    float gravity_component = gravity * sin(fabs(slope_angle));
    float friction_component = mu_kinetic * gravity * cos(fabs(slope_angle));
    
    float acceleration;
    if (abs(platform.rotation) > 5.0f) {
        // Sloped platform
        acceleration = gravity_component - friction_component;
    } else {
        // Horizontal platform - only friction (deceleration)
        acceleration = -friction_component;
    }
    
    // v² = u² + 2as
    float final_velocity_squared = initial_velocity * initial_velocity + 2 * acceleration * distance;
    
    if (final_velocity_squared <= 0) {
        return 0.0f; // Box stops
    }
    
    return sqrt(final_velocity_squared);
}


float Box::CalculateStoppingDistanceOnPlatform(float initial_velocity, const Platform& platform) {
    float slope_angle = platform.rotation * DEG2RAD;
    
    // Calculate acceleration on this platform
    float gravity_component = gravity * sin(fabs(slope_angle));
    float friction_component = mu_kinetic * gravity * cos(fabs(slope_angle));
    
    float acceleration;
    if (abs(platform.rotation) > 5.0f) {
        // Sloped platform - box accelerates down
        acceleration = gravity_component - friction_component;
        if (acceleration <= 0) {
            return 0.0f; // Won't slide, stops immediately
        }
        // On slopes, box typically won't stop due to friction alone unless very steep
        return std::numeric_limits<float>::max();
    } else {
        // Horizontal platform - only friction (deceleration)
        acceleration = -friction_component;
        if (acceleration >= 0 || initial_velocity <= 0) {
            return std::numeric_limits<float>::max(); // No deceleration or no initial velocity
        }
    }
    
    // v² = u² + 2as, solving for s when v = 0
    // 0 = u² + 2as → s = -u²/(2a)
    float stopping_distance = -(initial_velocity * initial_velocity) / (2.0f * acceleration);
    return std::max(0.0f, stopping_distance);
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
    float tolerance = 30.0f;
    const Platform* best_platform = nullptr;
    float min_distance = std::numeric_limits<float>::max();
    Vector2 current_right_end = GetRightEndOfPlatform(current_platform);
    
    for (const auto& platform : platforms) {
        if (&platform == &current_platform) continue;
        
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