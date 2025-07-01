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

void Object::CheckGrab() {
    Vector2 mouse_position = GetMousePosition();
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        if (CheckCollisionPointCircle(mouse_position, position, 20.0f)) { // radius of 20.0f for the grab area
            is_grabbed = true;
            grab_offset = Vector2Subtract(position, mouse_position);
            grab_position = position;
        }
    } else if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
        is_grabbed = false;
    }
}

void Object::Grab(Vector2 mouse_position) {
    if (is_grabbed) {
        position = Vector2Add(mouse_position, grab_offset);
        velocity = Vector2Subtract(position, grab_position);
        grab_position = position; // update grab position to current position
    }
}