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
        DrawText(TextFormat("%.2f deg", rotation), position.x + 25, position.y - 10, 20, BLACK);
    }
}