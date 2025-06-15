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
    // normalize vectors so they are same length
    Vector2 norm_velocity = Vector2Normalize(velocity);
    Vector2 norm_acceleration = Vector2Normalize(acceleration);
    // draw arrow from center of object to end of vector
    DrawLineEx(position, Vector2Add(position, Vector2Scale(norm_velocity, 50.0f)), 3.0f, PURPLE);
    DrawLineEx(position, Vector2Add(position, Vector2Scale(norm_acceleration, 50.0f)), 3.0f, YELLOW);
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