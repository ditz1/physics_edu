#include <box.hpp>
#include <iostream>

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
    
    DrawRectanglePro(
        (Rectangle){ position.x, position.y, size.x - border * 2, size.y - border * 2 }, 
        (Vector2){ (size.x - border * 2) * 0.5f, (size.y - border * 2) * 0.5f }, // origin at center of smaller rect
        rotation, 
        color
    );
    
    
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

void Box::CheckRaycastCollision(const std::vector<Platform>& platforms) {
    // Calculate rotated bottom corners
    float box_angle = rotation * DEG2RAD;
    float cos_rot = cosf(box_angle);
    float sin_rot = sinf(box_angle);
    
    // Local bottom corner positions (relative to box center)
    Vector2 local_bottom_left = {-size.x * 0.5f, size.y * 0.5f};
    Vector2 local_bottom_right = {size.x * 0.5f, size.y * 0.5f};
    
    // Transform to world coordinates
    Vector2 bottomLeft = {
        local_bottom_left.x * cos_rot - local_bottom_left.y * sin_rot + position.x,
        local_bottom_left.x * sin_rot + local_bottom_left.y * cos_rot + position.y
    };
    
    Vector2 bottomRight = {
        local_bottom_right.x * cos_rot - local_bottom_right.y * sin_rot + position.x,
        local_bottom_right.x * sin_rot + local_bottom_right.y * cos_rot + position.y
    };
    
    // Make raycast length dynamic based on velocity and add base length
    float dynamic_raycast_length = raycastLength + fabs(velocity.y) * GetFrameTime() + 5.0f;
    
    // Cast rays downward
    Vector2 leftRayEnd = {bottomLeft.x, bottomLeft.y + dynamic_raycast_length};
    Vector2 rightRayEnd = {bottomRight.x, bottomRight.y + dynamic_raycast_length};
    
    auto leftHits = CollisionUtils::RaycastToAllPlatforms(bottomLeft, leftRayEnd, platforms);
    auto rightHits = CollisionUtils::RaycastToAllPlatforms(bottomRight, rightRayEnd, platforms);
    
    // Get closest hits
    leftCornerHit = leftHits.empty() ? RaycastHit{false, {0,0}, {0,0}, 0, nullptr} : leftHits[0];
    rightCornerHit = rightHits.empty() ? RaycastHit{false, {0,0}, {0,0}, 0, nullptr} : rightHits[0];
    
    // Determine if we're colliding and how much to move
    bool shouldCollide = false;
    float moveDistance = 0;
    Platform* hitPlatform = nullptr;
    
    // Smaller collision threshold to reduce jerkiness
    float collision_threshold = 3.0f;
    
    if (leftCornerHit.hit && rightCornerHit.hit) {
        // Both corners hit something - use the closest
        float leftDistance = leftCornerHit.distance;
        float rightDistance = rightCornerHit.distance;
        
        if (leftDistance < collision_threshold || rightDistance < collision_threshold) {
            shouldCollide = true;
            if (leftDistance < rightDistance) {
                moveDistance = leftDistance;
                hitPlatform = leftCornerHit.platform;
            } else {
                moveDistance = rightDistance;
                hitPlatform = rightCornerHit.platform;
            }
        }
    } else if (leftCornerHit.hit && leftCornerHit.distance < collision_threshold) {
        shouldCollide = true;
        moveDistance = leftCornerHit.distance;
        hitPlatform = leftCornerHit.platform;
    } else if (rightCornerHit.hit && rightCornerHit.distance < collision_threshold) {
        shouldCollide = true;
        moveDistance = rightCornerHit.distance;
        hitPlatform = rightCornerHit.platform;
    }
    
    if (shouldCollide && hitPlatform) {
        is_colliding = true;
        
        // Gentle position correction - only move if we're actually penetrating
        if (moveDistance < 1.0f) {
            position.y -= (1.0f - moveDistance); // Move up just enough to get out
        }
        
        rotation = hitPlatform->rotation; // Set box rotation to match platform
        
        // Only stop downward velocity, preserve horizontal velocity for slopes
        if (velocity.y > 0) {
            velocity.y = 0;
        }
    } else {
        is_colliding = false;
        rotation = 0; // Reset rotation when not on platform
    }
}

void Box::DrawRaycasts() {
    if (!edit_mode) return;
    
    // Use the same rotated corner calculation as in CheckRaycastCollision
    float box_angle = rotation * DEG2RAD;
    float cos_rot = cosf(box_angle);
    float sin_rot = sinf(box_angle);
    
    Vector2 local_bottom_left = {-size.x * 0.5f, size.y * 0.5f};
    Vector2 local_bottom_right = {size.x * 0.5f, size.y * 0.5f};
    
    Vector2 bottomLeft = {
        local_bottom_left.x * cos_rot - local_bottom_left.y * sin_rot + position.x,
        local_bottom_left.x * sin_rot + local_bottom_left.y * cos_rot + position.y
    };
    
    Vector2 bottomRight = {
        local_bottom_right.x * cos_rot - local_bottom_right.y * sin_rot + position.x,
        local_bottom_right.x * sin_rot + local_bottom_right.y * cos_rot + position.y
    };
    
    float dynamic_raycast_length = raycastLength + fabs(velocity.y) * GetFrameTime() + 5.0f;
    Vector2 leftRayEnd = {bottomLeft.x, bottomLeft.y + dynamic_raycast_length};
    Vector2 rightRayEnd = {bottomRight.x, bottomRight.y + dynamic_raycast_length};
    
    // Draw rays
    DrawLineEx(bottomLeft, leftRayEnd, 2.0f, YELLOW);
    DrawLineEx(bottomRight, rightRayEnd, 2.0f, YELLOW);
    
    // Draw hit points
    if (leftCornerHit.hit) {
        DrawCircleV(leftCornerHit.point, 5.0f, GREEN);
        DrawText(TextFormat("L: %.1f", leftCornerHit.distance), 
                leftCornerHit.point.x - 20, leftCornerHit.point.y - 20, 12, GREEN);
    }
    
    if (rightCornerHit.hit) {
        DrawCircleV(rightCornerHit.point, 5.0f, GREEN);
        DrawText(TextFormat("R: %.1f", rightCornerHit.distance), 
                rightCornerHit.point.x + 10, rightCornerHit.point.y - 20, 12, GREEN);
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