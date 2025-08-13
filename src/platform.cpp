#include "platform.hpp"

void Platform::Draw() {
    float border = 4.0f;
    
    // Draw the base rectangle (for collision bounds visualization in edit mode)
    if (edit_mode) {
        DrawRectanglePro(
            (Rectangle){ position.x, position.y, size.x, size.y }, 
            (Vector2){ size.x * 0.5f, size.y * 0.5f }, // origin at center
            rotation, 
            Fade(BLACK, 0.3f)
        );
        
        DrawRectanglePro(
            (Rectangle){ position.x, position.y, size.x - border * 2, size.y - border * 2 }, 
            (Vector2){ (size.x - border * 2) * 0.5f, (size.y - border * 2) * 0.5f }, // origin at center of smaller rect
            rotation, 
            Fade(BEIGE, 0.3f)
        );
    }

    // NEW: Dynamic log texture drawing with slices
    if (log_texture != nullptr && log_end_texture != nullptr && log_slice_texture != nullptr) {
        DrawLogWithSlices();
    }
    // Fallback to old method if new textures aren't available
    else if (log_texture != nullptr) {
        // Old single texture method
        float scale_x = size.x / log_texture->width;
        float scale_y = size.y / log_texture->height;
        
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
    top_left = rotated_top_left;
    
    Vector2 rotated_top_right = {
        local_top_right.x * cos_rot - local_top_right.y * sin_rot + position.x,
        local_top_right.x * sin_rot + local_top_right.y * cos_rot + position.y
    };
    top_right = rotated_top_right;

    
    // Draw measurement line across the rotated top edge
    if (edit_mode) {
        DrawLineEx(rotated_top_left, rotated_top_right, 2.0f, DARKGRAY);
        
        // Draw ruler markings
        float mark_spacing = 20.0f;
        float mark_length = 10.0f;
        
        Vector2 edge_dir = { 
            rotated_top_right.x - rotated_top_left.x, 
            rotated_top_right.y - rotated_top_left.y 
        };
        float edge_length = sqrtf(edge_dir.x * edge_dir.x + edge_dir.y * edge_dir.y);
        
        edge_dir.x /= edge_length;
        edge_dir.y /= edge_length;
        
        Vector2 perp_dir = { edge_dir.y, -edge_dir.x };
        
        int num_marks = (int)(edge_length / mark_spacing);
        for (int i = 0; i <= num_marks; i++) {
            float t = (float)i / num_marks;
            
            Vector2 mark_pos = {
                rotated_top_left.x + edge_dir.x * (t * edge_length),
                rotated_top_left.y + edge_dir.y * (t * edge_length)
            };
            
            Vector2 mark_end = {
                mark_pos.x + perp_dir.x * -mark_length,
                mark_pos.y + perp_dir.y * -mark_length
            };
            
            if (i > 0){
                DrawLineEx(mark_pos, mark_end, 2.0f, GRAY);
                DrawTextPro(GetFontDefault(), TextFormat("%d", i), (Vector2) {mark_pos.x - 10, mark_pos.y + 10}, (Vector2){ 0, 0 }, rotation, 10.0f, 1.0f, BLACK);
            }
        }
        
        DrawCircleV(position, 20.0f, RED);
        DrawText(TextFormat("%.2f deg [%d]", rotation, id), position.x + 25, position.y - 10, 20, BLACK);
        
        Vector2 resize_handle = GetResizeHandlePosition();
        Color handle_color = is_resizing ? ORANGE : YELLOW;
        DrawCircleV(resize_handle, 8.0f, handle_color);
        DrawCircleLines(resize_handle.x, resize_handle.y, 8.0f, BLACK);
        
        if (is_resizing) {
            Vector2 anchor_point = GetAnchorPosition();
            DrawCircleV(anchor_point, 6.0f, GREEN);
            DrawText("ANCHOR", anchor_point.x - 25, anchor_point.y - 20, 12, GREEN);
        }
    } else {
        if (rotation != 0.0f) {
            Vector2 text_pos = {position.x - 20, position.y + 20};
            DrawTextBg(TextFormat("%.2f deg", rotation), text_pos, 16, BLACK, (Color){200,200,200,220});
        }
    }
}

void Platform::DrawLogWithSlices() {
    // Get texture dimensions
    float start_width = (float)log_texture->width;
    float end_width = (float)log_end_texture->width;
    float slice_width = (float)log_slice_texture->width;
    
    // Calculate how many middle slices we need
    float available_length = size.x - start_width - end_width;
    int num_middle_slices = fmaxf(1, (int)(available_length / slice_width));
    float actual_slice_width = available_length / num_middle_slices;
    
    // Calculate platform direction
    float cos_rot = cosf(rotation * DEG2RAD);
    float sin_rot = sinf(rotation * DEG2RAD);
    
    // Start position (left end of platform)
    Vector2 start_pos = {
        position.x - (size.x * 0.5f) * cos_rot,
        position.y - (size.x * 0.5f) * sin_rot
    };
    
    float current_x_offset = 0.0f;
    
    // 1. Draw start piece
    Vector2 start_piece_pos = {
        start_pos.x + (current_x_offset + start_width * 0.5f) * cos_rot,
        start_pos.y + (current_x_offset + start_width * 0.5f) * sin_rot
    };
    
    DrawTexturePro(
        *log_texture,
        (Rectangle){0, 0, (float)log_texture->width, (float)log_texture->height}, // Full start texture
        (Rectangle){start_piece_pos.x, start_piece_pos.y, start_width, size.y}, // Destination
        (Vector2){start_width * 0.5f, size.y * 0.5f}, // Origin at center
        rotation,
        WHITE
    );
    
    current_x_offset += start_width;
    
    // 2. Draw middle slices
    for (int i = 0; i < num_middle_slices; i++) {
        Vector2 slice_piece_pos = {
            start_pos.x + (current_x_offset + actual_slice_width * 0.5f) * cos_rot,
            start_pos.y + (current_x_offset + actual_slice_width * 0.5f) * sin_rot
        };
        
        DrawTexturePro(
            *log_slice_texture,
            (Rectangle){0, 0, (float)log_slice_texture->width, (float)log_slice_texture->height}, // Full slice texture
            (Rectangle){slice_piece_pos.x, slice_piece_pos.y, actual_slice_width, size.y}, // Destination
            (Vector2){actual_slice_width * 0.5f, size.y * 0.5f}, // Origin at center
            rotation,
            WHITE
        );
        
        current_x_offset += actual_slice_width;
    }
    
    // 3. Draw end piece
    Vector2 end_piece_pos = {
        start_pos.x + (current_x_offset + end_width * 0.5f) * cos_rot,
        start_pos.y + (current_x_offset + end_width * 0.5f) * sin_rot
    };
    
    DrawTexturePro(
        *log_end_texture,
        (Rectangle){0, 0, (float)log_end_texture->width, (float)log_end_texture->height}, // Full end texture
        (Rectangle){end_piece_pos.x, end_piece_pos.y, end_width, size.y}, // Destination
        (Vector2){end_width * 0.5f, size.y * 0.5f}, // Origin at center
        rotation,
        WHITE
    );
}

// Rest of the methods remain the same...
Vector2 Platform::GetResizeHandlePosition() const {
    float cos_rot = cosf(rotation * DEG2RAD);
    float sin_rot = sinf(rotation * DEG2RAD);
    
    Vector2 local_bottom_right = { size.x * 0.5f, size.y * 0.5f };
    
    Vector2 world_bottom_right = {
        local_bottom_right.x * cos_rot - local_bottom_right.y * sin_rot + position.x,
        local_bottom_right.x * sin_rot + local_bottom_right.y * cos_rot + position.y
    };
    
    return world_bottom_right;
}

Vector2 Platform::GetAnchorPosition() const {
    if (!is_resizing) {
        float cos_rot = cosf(rotation * DEG2RAD);
        float sin_rot = sinf(rotation * DEG2RAD);
        
        Vector2 local_top_left = { -size.x * 0.5f, -size.y * 0.5f };
        
        Vector2 world_top_left = {
            local_top_left.x * cos_rot - local_top_left.y * sin_rot + position.x,
            local_top_left.x * sin_rot + local_top_left.y * cos_rot + position.y
        };
        
        return world_top_left;
    } else {
        return resize_anchor_point;
    }
}

void Platform::CheckResize(Vector2 world_mouse_position) {
    if (!edit_mode) return;
    
    Vector2 resize_handle = GetResizeHandlePosition();
    
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        if (CheckCollisionPointCircle(world_mouse_position, resize_handle, 8.0f)) {
            is_resizing = true;
            resize_start_size = size;
            resize_start_position = position;
            resize_start_mouse = world_mouse_position;  // Store world coordinates
            
            float cos_rot = cosf(rotation * DEG2RAD);
            float sin_rot = sinf(rotation * DEG2RAD);
            
            Vector2 local_top_left = { -size.x * 0.5f, -size.y * 0.5f };
            
            resize_anchor_point = {
                local_top_left.x * cos_rot - local_top_left.y * sin_rot + position.x,
                local_top_left.x * sin_rot + local_top_left.y * cos_rot + position.y
            };
            
            is_grabbed = false;
        }
    } else if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
        is_resizing = false;
    }
}

void Platform::HandleResize(Vector2 world_mouse_position) {
    if (!is_resizing) return;
    
    // Rest of the function uses world_mouse_position instead of mouse_position
    float cos_rot = cosf(-rotation * DEG2RAD);
    float sin_rot = sinf(-rotation * DEG2RAD);
    
    Vector2 mouse_relative = Vector2Subtract(world_mouse_position, resize_start_position);
    
    // Rest of the function remains the same...
    Vector2 mouse_local = {
        mouse_relative.x * cos_rot - mouse_relative.y * sin_rot,
        mouse_relative.x * sin_rot + mouse_relative.y * cos_rot
    };
    
    Vector2 local_anchor = { -resize_start_size.x * 0.5f, -resize_start_size.y * 0.5f };
    Vector2 local_bottom_right = mouse_local;
    Vector2 local_size_vector = Vector2Subtract(local_bottom_right, local_anchor);
    
    Vector2 new_size = {
        fabs(local_size_vector.x),
        fabs(local_size_vector.y)
    };
    
    new_size.x = fmax(20.0f, new_size.x);
    new_size.y = fmax(20.0f, new_size.y);
    
    Vector2 local_new_center;
    
    if (local_size_vector.x >= 0 && local_size_vector.y >= 0) {
        local_new_center = {
            local_anchor.x + new_size.x * 0.5f,
            local_anchor.y + new_size.y * 0.5f
        };
    } else if (local_size_vector.x < 0 && local_size_vector.y >= 0) {
        local_new_center = {
            local_anchor.x - new_size.x * 0.5f,
            local_anchor.y + new_size.y * 0.5f
        };
    } else if (local_size_vector.x >= 0 && local_size_vector.y < 0) {
        local_new_center = {
            local_anchor.x + new_size.x * 0.5f,
            local_anchor.y - new_size.y * 0.5f
        };
    } else {
        local_new_center = {
            local_anchor.x - new_size.x * 0.5f,
            local_anchor.y - new_size.y * 0.5f
        };
    }
    
    float cos_rot_forward = cosf(rotation * DEG2RAD);
    float sin_rot_forward = sinf(rotation * DEG2RAD);
    
    Vector2 world_center_offset = {
        local_new_center.x * cos_rot_forward - local_new_center.y * sin_rot_forward,
        local_new_center.x * sin_rot_forward + local_new_center.y * cos_rot_forward
    };
    
    Vector2 new_world_center = Vector2Add(resize_start_position, world_center_offset);
    
    size = new_size;
    position = new_world_center;
}