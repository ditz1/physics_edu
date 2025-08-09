#include "camera.hpp"

SimCamera::SimCamera() {
    camera.target = {0, 0};
    camera.offset = {0, 0};
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;
    
    // Initialize bounds to extreme values
    leftmost_x = 10000;
    rightmost_x = -10000;
    topmost_y = 10000;
    bottommost_y = -10000;
}

void SimCamera::Update(float dt) {
    // Camera update logic can go here if needed (smooth transitions, etc.)
}

void SimCamera::FindBounds(std::vector<Platform> platforms) {
    if (platforms.empty()) return;
    
    // Reset bounds to extreme values
    leftmost_x = 10000;
    rightmost_x = -10000;
    topmost_y = 10000;
    bottommost_y = -10000;

    // Find the actual bounds by checking all platform corners
    for (const Platform& plat : platforms) {
        // Calculate all four corners of the rotated platform
        float cos_rot = cosf(plat.rotation * DEG2RAD);
        float sin_rot = sinf(plat.rotation * DEG2RAD);
        
        // Local corners relative to center
        Vector2 corners[4] = {
            { -plat.size.x * 0.5f, -plat.size.y * 0.5f }, // Top-left
            {  plat.size.x * 0.5f, -plat.size.y * 0.5f }, // Top-right
            {  plat.size.x * 0.5f,  plat.size.y * 0.5f }, // Bottom-right
            { -plat.size.x * 0.5f,  plat.size.y * 0.5f }  // Bottom-left
        };
        
        // Transform each corner to world coordinates and update bounds
        for (int i = 0; i < 4; i++) {
            Vector2 world_corner = {
                corners[i].x * cos_rot - corners[i].y * sin_rot + plat.position.x,
                corners[i].x * sin_rot + corners[i].y * cos_rot + plat.position.y
            };
            
            if (world_corner.x < leftmost_x) leftmost_x = world_corner.x;
            if (world_corner.x > rightmost_x) rightmost_x = world_corner.x;
            if (world_corner.y < topmost_y) topmost_y = world_corner.y;
            if (world_corner.y > bottommost_y) bottommost_y = world_corner.y;
        }
    }
    
    // Add some padding around the bounds
    float padding = 100.0f;
    leftmost_x -= padding;
    rightmost_x += padding;
    topmost_y -= padding;
    bottommost_y += padding;
    
    // Calculate the center of the bounding area
    float center_x = (leftmost_x + rightmost_x) * 0.5f;
    float center_y = (topmost_y + bottommost_y) * 0.5f;
    
    // Calculate required dimensions
    float required_width = rightmost_x - leftmost_x;
    float required_height = bottommost_y - topmost_y;
    
    // Get screen dimensions
    float screen_width = (float)GetScreenWidth();
    float screen_height = (float)GetScreenHeight();
    
    // Calculate zoom to fit both width and height
    float zoom_x = screen_width / required_width;
    float zoom_y = screen_height / required_height;
    
    // Use the smaller zoom to ensure everything fits
    float target_zoom = fminf(zoom_x, zoom_y);
    
    // Clamp zoom to reasonable limits
    target_zoom = fmaxf(0.1f, fminf(target_zoom, 3.0f));
    
    // Set camera properties
    camera.target = { center_x, center_y };
    camera.offset = { screen_width * 0.5f, screen_height * 0.5f };
    camera.zoom = target_zoom;    
}