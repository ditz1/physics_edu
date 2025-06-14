#include <spring.hpp>


Spring::Spring() {
    // TODO;
}

Spring::~Spring() {
    // TODO;
}

void Spring::Draw() {
    // Calculate current length and compression ratio
    float current_length = Vector2Distance(anchor, position);
    float compression_ratio = (rest_length - current_length) / rest_length;
    
    // Clamp the ratio to reasonable bounds
    compression_ratio = fmaxf(-1.0f, fminf(1.0f, compression_ratio));
    
    Color line_color;
    
    if (compression_ratio > 0) {
        // Compressed: interpolate from blue to red
        float red_factor = compression_ratio;
        line_color = (Color){
            (unsigned char)(0 + red_factor * 255),      // Red component
            0,                                          // Green component  
            (unsigned char)(255 - red_factor * 255),    // Blue component
            255                                         // Alpha
        };
    } else {
        // Extended: interpolate from blue to cyan/white or another color
        float extend_factor = -compression_ratio;
        line_color = (Color){
            0,                                          // Red component
            (unsigned char)(extend_factor * 100),       // Green component (adjust as needed)
            255,                                        // Blue component stays high
            255                                         // Alpha
        };
    }
    
    DrawLineEx(anchor, position, 6.0f, line_color);
    
    DrawCircleV(position, radius, RED);
    DrawCircleV(anchor, 10.0f, BLACK);
}

void Spring::Update(float dt) {
    // F = -k * x

    Vector2 force = Vector2Subtract(anchor, position);
    float distance = Vector2Length(force);
    float x = distance - rest_length;
    _compression = x;
    force = Vector2Scale(Vector2Normalize(force), spring_constant * x);
    // spring constant cant be negative here because of screen coordinate system,
    // Y axis is increasing downwards, so if spring const was negative would be messed up
    
    force = Vector2Add(force, Vector2Scale(velocity, -damping_factor));
    // damping factor may not be absolutely necessary, but it helps to make the simulation more stable
    
    _force = force; // for debugging purposes, should not be used directly
  
    // apply forces to spring
    acceleration = Vector2Scale(force, 1.0f / mass);
    velocity = Vector2Add(velocity, Vector2Scale(acceleration, dt));
    position = Vector2Add(position, Vector2Scale(velocity, dt));
   
    ApplyGravity(dt);
}


