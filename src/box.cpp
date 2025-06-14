#include <box.hpp>


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
    if (position.y + size.y < GetScreenHeight()) {
        ApplyGravity(dt);
    }
    ApplyFriction(dt);

    position = Vector2Add(position, Vector2Scale(velocity, dt));
}

void Box::Draw() {
    DrawRectangleV(position, size, color);
}

void Box::CheckCollision() {
    int s_width = GetScreenWidth();
    int s_height = GetScreenHeight();

    if (position.x < 0.0f || position.x + size.x > s_width) {
        velocity.x *= -0.2f; // reverse x velocity
        position.x = (position.x < 0.0f) ? 0.0f : s_width - size.x; // reset position to screen bounds
    }
    if (position.y < 0.0f || position.y + size.y > s_height) {
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

void Box::ApplyFriction(float dt) {
    if (position.y + size.y >= GetScreenHeight()) {
        float normal_force = mass * gravity; 
        // pretty sure this is the normal force
        // since we just expect box to hit the ground,
        // but this should change if its on an incline which eventually will be the case


        // only applying friction with horizontal movement for now just to see that it works
        if (abs(velocity.x) > 0.001f) { // if box is moving at all
            float friction_force = mu_kinetic * normal_force;
            float friction_accel = friction_force / mass;
            
            // use max and min here because the friction is a force at the end of the day,
            // meaning that it could override the velocity and the box could start moving in the opposite direction
            // which is unrealistic (?) so should be clamped to zero
            if (velocity.x > 0) {
                velocity.x = fmax(0.0f, velocity.x - friction_accel * dt);
            } else {
                velocity.x = fmin(0.0f, velocity.x + friction_accel * dt);
            }
        }
    }
}