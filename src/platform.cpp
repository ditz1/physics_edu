#include "platform.hpp"


void Platform::Draw() {
    float border = 4.0f;
    
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
        BEIGE
    );

    // UPDATED: Calculate scale to stretch log texture to fit platform
    if (log_texture != nullptr) {
        // Calculate scale factors for width and height
        float scale_x = size.x / log_texture->width;
        float scale_y = size.y / log_texture->height;
        
        // Position the texture at the platform center, accounting for scaling
        Vector2 texture_position = {
            position.x - (log_texture->width * scale_x) * 0.5f,
            position.y - (log_texture->height * scale_y) * 0.5f
        };
        
        // Draw texture stretched to platform size with rotation
        DrawTexturePro(
            *log_texture,
            (Rectangle){0, 0, (float)log_texture->width, (float)log_texture->height}, // source
            (Rectangle){position.x, position.y - (size.y * 0.1f), size.x, size.y * 1.1f}, // destination
            (Vector2){size.x * 0.5f, size.y * 0.5f}, // origin at center
            rotation,
            WHITE
        );
    }

    // Calculate rotated top-left and top-right corners
    float cos_rot = cosf(rotation * DEG2RAD);
    float sin_rot = sinf(rotation * DEG2RAD);
    
    Vector2 local_top_left = { -size.x * 0.5f, -size.y * 0.5f };
    Vector2 local_top_right = { size.x * 0.5f, -size.y * 0.5f };
    
    Vector2 rotated_top_left = {
        local_top_left.x * cos_rot - local_top_left.y * sin_rot + position.x,
        local_top_left.x * sin_rot + local_top_left.y * cos_rot + position.y
    };
    top_left = rotated_top_left; // Store for later use
    
    Vector2 rotated_top_right = {
        local_top_right.x * cos_rot - local_top_right.y * sin_rot + position.x,
        local_top_right.x * sin_rot + local_top_right.y * cos_rot + position.y
    };
    top_right = rotated_top_right; // Store for later use
    
    // Draw measurement line across the rotated top edge
    DrawLineEx(rotated_top_left, rotated_top_right, 2.0f, DARKGRAY);
    
    // Draw ruler markings - perpendicular lines going INTO the rectangle
    float mark_spacing = 20.0f; // Distance between marks
    float mark_length = 10.0f;   // Length of each mark
    
    // Direction vector along the top edge
    Vector2 edge_dir = { 
        rotated_top_right.x - rotated_top_left.x, 
        rotated_top_right.y - rotated_top_left.y 
    };
    float edge_length = sqrtf(edge_dir.x * edge_dir.x + edge_dir.y * edge_dir.y);
    
    // Normalize the edge direction
    edge_dir.x /= edge_length;
    edge_dir.y /= edge_length;
    
    // Perpendicular direction pointing INTO the rectangle (rotate -90 degrees)
    Vector2 perp_dir = { edge_dir.y, -edge_dir.x };
    
    // Draw marks along the edge
    int num_marks = (int)(edge_length / mark_spacing);
    for (int i = 0; i <= num_marks; i++) {
        float t = (float)i / num_marks;
        
        // Position along the edge
        Vector2 mark_pos = {
            rotated_top_left.x + edge_dir.x * (t * edge_length),
            rotated_top_left.y + edge_dir.y * (t * edge_length)
        };
        
        // Draw perpendicular mark going INTO the rectangle
        Vector2 mark_end = {
            mark_pos.x + perp_dir.x * -mark_length,
            mark_pos.y + perp_dir.y * -mark_length
        };
        
       
        //void DrawTextPro(Font font, const char *text, Vector2 position, Vector2 origin, float rotation, float fontSize, float spacing, Color tint)
        if (i > 0){
            DrawLineEx(mark_pos, mark_end, 2.0f, GRAY);
            DrawTextPro(GetFontDefault(), TextFormat("%d", i), (Vector2) {mark_pos.x - 10, mark_pos.y + 10}, (Vector2){ 0, 0 }, rotation, 10.0f, 1.0f, BLACK);
        }
    }
    
    if (edit_mode){
        DrawCircleV(position, 20.0f, RED);
        DrawText(TextFormat("%.2f deg [%d]", rotation, id), position.x + 25, position.y - 10, 20, BLACK);
        
        // Draw resize handle on bottom right corner
        Vector2 resize_handle = GetResizeHandlePosition();
        Color handle_color = is_resizing ? ORANGE : YELLOW;
        DrawCircleV(resize_handle, 8.0f, handle_color);
        DrawCircleLines(resize_handle.x, resize_handle.y, 8.0f, BLACK);
    }
}

Vector2 Platform::GetResizeHandlePosition() const {
    // Calculate bottom right corner position accounting for rotation
    float cos_rot = cosf(rotation * DEG2RAD);
    float sin_rot = sinf(rotation * DEG2RAD);
    
    // Local bottom right corner relative to center
    Vector2 local_bottom_right = { size.x * 0.5f, size.y * 0.5f };
    
    // Rotate and translate to world coordinates
    Vector2 world_bottom_right = {
        local_bottom_right.x * cos_rot - local_bottom_right.y * sin_rot + position.x,
        local_bottom_right.x * sin_rot + local_bottom_right.y * cos_rot + position.y
    };
    
    return world_bottom_right;
}

void Platform::CheckResize() {
    if (!edit_mode) return;
    
    Vector2 mouse_position = GetMousePosition();
    Vector2 resize_handle = GetResizeHandlePosition();
    
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        // Check if mouse is clicking on resize handle
        if (CheckCollisionPointCircle(mouse_position, resize_handle, 8.0f)) {
            is_resizing = true;
            resize_start_size = size;
            resize_start_mouse = mouse_position;
            // Prevent normal grab from triggering
            is_grabbed = false;
        }
    } else if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
        is_resizing = false;
    }
}

void Platform::HandleResize(Vector2 mouse_position) {
    if (!is_resizing) return;
    
    // Calculate distance from platform center to mouse position
    Vector2 center_to_mouse = Vector2Subtract(mouse_position, position);
    float distance_to_mouse = Vector2Length(center_to_mouse);
    
    // Calculate the initial distance from center to resize handle when resize started
    Vector2 center_to_start = Vector2Subtract(resize_start_mouse, position);
    float initial_distance = Vector2Length(center_to_start);
    
    // Calculate scale factor based on how much the mouse moved relative to initial distance
    float scale_factor = (initial_distance > 0) ? (distance_to_mouse / initial_distance) : 1.0f;
    
    // Apply the scale factor to the original size, with minimum constraints
    Vector2 new_size = {
        fmax(20.0f, resize_start_size.x * scale_factor),
        fmax(20.0f, resize_start_size.y * scale_factor)
    };
    
    size = new_size;
}