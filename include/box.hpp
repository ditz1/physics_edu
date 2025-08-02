#pragma once
#include <object.hpp>
#include "platform.hpp"

struct TrajectorySegment {
    Vector2 start_position;
    Vector2 end_position;
    const Platform* platform;
    float distance;
};



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
    int transition_cooldown = 0; // Prevent rapid platform transitions

    Vector2 prediction_start_position;
    bool has_prediction_start = false;
    bool ghost_calculated = false;
    Vector2 ghost_position_stored;
    Vector2 transition_point_stored;
    float distance_traveled = 0.0f;
    Texture2D* texture;

    std::vector<TrajectorySegment> trajectory_segments;
    Vector2 final_ghost_position;
    bool multi_platform_ghost_calculated = false;

    void Update(float dt, const std::vector<Platform>& platforms = {});
    void Draw() override;
    void CheckCollision();
    void CheckPlatformCollisionSAT(const Platform& platform, int platform_id);
    void CheckPlatformCollisionTwoLine(const std::vector<Platform>& platforms);
    void ApplyFriction(float dt);
    void SetPredictionStartPosition();
    void CalculateGhostTrajectory(const std::vector<Platform>& platforms);
    bool IsPointOnPlatform(Vector2 point, const Platform& platform);
    float GetDistanceToEndOfPlatform(Vector2 start_pos, const Platform& platform);
    Vector2 GetPlatformDirection(const Platform& platform);
    float CalculateFinalVelocity(float initial_velocity, const Platform& platform, float distance);
    float CalculateStoppingDistanceOnPlatform(float initial_velocity, const Platform& platform);
    float CalculateDistanceToConnection(Vector2 start_pos, const Platform& current_platform, const Platform& next_platform);
    const Platform* FindNextPlatform(Vector2 current_pos, const std::vector<Platform>& platforms, const Platform* current_platform);
    const Platform* FindNextConnectedPlatform(const Platform& current_platform, const std::vector<Platform>& platforms);
    Vector2 GetRightEndOfPlatform(const Platform& platform);
    const Platform* FindNextPlatformToRight(const Platform& current_platform, const std::vector<Platform>& platforms);

    float CalculateStoppingDistanceFromSlope(const Platform& slope_platform, float slope_travel_distance);
    float GetSlopeDistance(const Platform& platform);
    float GetDistanceAlongSlope(Vector2 start_point, Vector2 end_point, const Platform& slope_platform);
    Vector2 CalculateStoppingPosition(const Platform& slope_platform, const Platform& horizontal_platform);
    void DrawGhost(const Platform& slope_platform, const Platform& horizontal_platform);
    void DrawMultiPlatformGhost(const std::vector<Platform>& platforms);
    void CalculateMultiPlatformTrajectory(const std::vector<Platform>& platforms);
    void DrawTwoLineCollisionDebug(const std::vector<Platform>& platforms);

    Rectangle inline Rect() const {
        return (Rectangle){ position.x, position.y, size.x, size.y };
    }
    void CheckPlatformCollision(Rectangle platform_rect);
    
};