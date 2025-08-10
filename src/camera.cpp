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
    // (Intentionally minimal; smoothing is handled in FindBounds so you don't
    // have to change the rest of your codebase or pass dt through more places.)
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
    float desired_center_x = (leftmost_x + rightmost_x) * 0.5f;
    float desired_center_y = (topmost_y + bottommost_y) * 0.5f;
    
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
    float desired_zoom = fminf(zoom_x, zoom_y);
    
    // Clamp zoom to reasonable limits
    desired_zoom = fmaxf(0.1f, fminf(desired_zoom, 3.0f));
    
    // Keep camera offset centered on the screen
    camera.offset = { screen_width * 0.5f, screen_height * 0.5f };

    // --- Smoothly move/zoom toward the desired frame ---
    // Use frame-time-based exponential smoothing so it feels consistent.
    // Higher 'speed' values converge faster.
    float dt = GetFrameTime();
    float follow_speed = 6.0f; // position smoothing speed
    float zoom_speed = 6.0f;   // zoom smoothing speed

    // First-time init: snap immediately so we don't see a big jump on first frame
    static bool initialized = false;
    if (!initialized) {
        camera.target = { desired_center_x, desired_center_y };
        camera.zoom = desired_zoom;
        initialized = true;
        return;
    }

    // Convert the speed constants into a [0..1] interpolation factor per frame
    float a_pos = 1.0f - expf(-follow_speed * dt);
    float a_zoom = 1.0f - expf(-zoom_speed * dt);

    // Lerp target
    camera.target.x += (desired_center_x - camera.target.x) * a_pos;
    camera.target.y += (desired_center_y - camera.target.y) * a_pos;

    // Lerp zoom
    camera.zoom += (desired_zoom - camera.zoom) * a_zoom;
}
