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
    
    // Draw with DrawRectanglePro - see where it appears
    DrawRectanglePro(
        (Rectangle){position.x, position.y, size.x, size.y}, 
        (Vector2){ size.x * 0.5f, size.y * 0.5f }, 
        rotation, 
        BLUE  // Different color to see both
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