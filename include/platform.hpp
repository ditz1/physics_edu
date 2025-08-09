#pragma once
#include "object.hpp" 

// platforms are static objects that can be used as surfaces for other objects to interact with
// they are collidable, but should never move
class Platform : public Object {
public:
    Platform(Vector2 position, Vector2 size, float rotation = 0.0f) {
        this->position = position; // This will be the center
        this->size = size;
        this->mass = 1.0f;
        this->is_grabbed = false;
        this->rotation = rotation;
        this->log_texture = nullptr; // Initialize to nullptr to prevent segfault
    }
    Vector2 size;
    float rotation = 0.0f;
    bool is_selected = false;
    Vector2 top_left; // after rotated
    Vector2 top_right; // after rotated
    int id = 0;
    
    // Resize functionality
    bool is_resizing = false;
    Vector2 resize_start_size;
    Vector2 resize_start_position;
    Vector2 resize_start_mouse;
    Vector2 resize_anchor_point; // NEW: Store the anchor point during resize
    Texture2D* log_texture;

    void Draw() override;
    void Update(float dt) override {}
    
    // Resize functionality methods
    void CheckResize();
    void HandleResize(Vector2 mouse_position);
    Vector2 GetResizeHandlePosition() const;
    Vector2 GetAnchorPosition() const; // NEW: Get the anchor point position
    
    // Helper function to get top-left corner if needed
    Vector2 GetTopLeft() const {
        return { position.x - size.x * 0.5f, position.y - size.y * 0.5f };
    }
};