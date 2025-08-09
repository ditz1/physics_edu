#include <object.hpp>

void Object::Update(float dt) {
    // meant to be overridden by derived classes
}

void Object::Draw() {
    // meant to be overridden by derived classes
}



void Object::ApplyGravity(float dt) {
    acceleration.y += 9.81f; // Gravity
    velocity = Vector2Add(velocity, Vector2Scale(acceleration, dt));
    float terminal_velocity = 500.0f; // Arbitrary terminal velocity limit
    if (velocity.y > terminal_velocity) {
        velocity.y = terminal_velocity; // Cap the velocity to terminal velocity
    }
}

void Object::DrawVectors() {
    float velocity_magnitude = Vector2Length(velocity);
    float acceleration_magnitude = Vector2Length(acceleration);
    
    if (velocity_magnitude > 0.01f) { // Small threshold to avoid division by zero
        Vector2 norm_velocity = Vector2Scale(velocity, 1.0f / velocity_magnitude);
        DrawLineEx(position, Vector2Add(position, Vector2Scale(norm_velocity, 50.0f)), 3.0f, PURPLE);
    }
    
    if (acceleration_magnitude > 0.01f) {
        Vector2 norm_acceleration = Vector2Scale(acceleration, 1.0f / acceleration_magnitude);
        DrawLineEx(position, Vector2Add(position, Vector2Scale(norm_acceleration, 50.0f)), 3.0f, YELLOW);
    }
}

void Object::CheckGrab(Vector2 world_mouse_position) {
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        if (CheckCollisionPointCircle(world_mouse_position, position, 20.0f)) {
            is_grabbed = true;
            grab_offset = Vector2Subtract(position, world_mouse_position);
            grab_position = position;
        }
    } else if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
        is_grabbed = false;
    }
}

void Object::Grab(Vector2 world_mouse_position) {
    if (is_grabbed) {
        // Calculate target position
        Vector2 target_position = Vector2Add(world_mouse_position, grab_offset);
        
        // Apply damping - only move a fraction of the distance toward target
        float damping_factor = 0.03f; ///t this value: 0.05f = very slow, 0.5f = fast
        Vector2 movement = Vector2Scale(Vector2Subtract(target_position, position), damping_factor);
        
        position = Vector2Add(position, movement);
        velocity = Vector2Subtract(position, grab_position);
        grab_position = position;
    }
}