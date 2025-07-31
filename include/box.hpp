#pragma once
#include <object.hpp>
#include "platform.hpp"

class Box : public Object {
public:
    Box();
    Box(int w, int h);
    ~Box();

    Vector2 size; // size of the box (width, height)
    Color color = RED;
    float mu_kinetic = 0.5f; // kinetic friction, 0.1 low friction, 1.0 high friction
    bool is_colliding = false; // used to check if the box is colliding with something
    float rotation = 0.0f;
    bool was_colliding_last_frame = false;
    int current_platform_id = -1;
    int last_platform_id = -1;

    Vector2 prediction_start_position;
    bool has_prediction_start = false;
    bool ghost_calculated = false;
    Vector2 ghost_position_stored;
    Vector2 transition_point_stored;
    float distance_traveled = 0.0f;

    // Raycast collision system
    RaycastHit leftCornerHit;
    RaycastHit rightCornerHit;
    float raycastLength = 100.0f; // How far down to cast rays

    void Update(float dt) override;
    void Draw() override;
    void CheckCollision();
    void CheckPlatformCollisionSAT(const Platform& platform, int platform_id);    
    void CheckRaycastCollision(const std::vector<Platform>& platforms);
    void DrawRaycasts(); // For debugging
    void ApplyFriction(float dt);
    void SetPredictionStartPosition();

    // Add these method declarations to your Box class
    float CalculateStoppingDistanceFromSlope(const Platform& slope_platform, float slope_travel_distance);
    float GetSlopeDistance(const Platform& platform);
    Vector2 CalculateStoppingPosition(const Platform& slope_platform, const Platform& horizontal_platform);
    void DrawGhost(const Platform& slope_platform, const Platform& horizontal_platform);

    Rectangle inline Rect() const {
        return (Rectangle){ position.x, position.y, size.x, size.y };
    }
    void CheckPlatformCollision(Rectangle platform_rect);
};