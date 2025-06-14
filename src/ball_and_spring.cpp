#include <ball_and_string.hpp>


// In ball_and_string.cpp
void BallAndString::Update(float dt) {
    if (!is_broken) {
        // Normal circular motion
        angle += angularSpeed * dt;
        if (angle > 2 * PI) angle -= 2 * PI;
        position = Position();
        
        // Set velocity to tangential velocity for when it breaks
        velocity = {
            -radius * angularSpeed * sin(angle),
            radius * angularSpeed * cos(angle)
        };
    } else {
        // Ball is broken free - use physics
        ApplyGravity(dt); // Apply gravity
        position = Vector2Add(position, Vector2Scale(velocity, dt));
        velocity = Vector2Add(velocity, Vector2Scale(acceleration, dt));
        acceleration = {0, 0}; // Reset acceleration after applying
    }
}

Vector2 BallAndString::Position() {
    if (!is_broken) {
        float x = anchor.x + radius * cos(angle);
        float y = anchor.y + radius * sin(angle);
        return {x, y};
    } else {
        return position; // Use current position when broken
    }
}

void BallAndString::Draw() {
    Vector2 pos = Position();
    DrawLineV(anchor, pos, WHITE);
    DrawCircleV(pos, 20.0f, BLUE);
    DrawCircleV(anchor, 5.0f, RED);
}

void BallAndString::DrawVectors() {
    if (!is_broken) {
        // Circular motion vectors
        position = Position();
        
        Vector2 tangent_velocity = {
            -radius * angularSpeed * sin(angle),
            radius * angularSpeed * cos(angle)
        };
        
        Vector2 centripetal_acc = {
            -radius * angularSpeed * angularSpeed * cos(angle),
            -radius * angularSpeed * angularSpeed * sin(angle)
        };
        
        Vector2 vel_end = Vector2Add(position, Vector2Scale(Vector2Normalize(tangent_velocity), 50.0f));
        DrawLineEx(position, vel_end, 3.0f, PURPLE);
        
        Vector2 acc_end = Vector2Add(position, Vector2Scale(Vector2Normalize(centripetal_acc), 50.0f));
        DrawLineEx(position, acc_end, 3.0f, YELLOW);
    } else {
        // Free-fall vectors (use parent class method)
        Object::DrawVectors();
    }
}

void BallAndString::Break() {
    if (!is_broken) {
        is_broken = true;
        
        // Set the ball's initial velocity to its tangential velocity at break
        velocity = {
            -radius * angularSpeed * sin(angle),
            radius * angularSpeed * cos(angle)
        };
        
        // Set position to current circular position
        position = Position();
    }
}