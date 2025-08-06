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
    std::vector<Vector2> projectile_trajectory_points; // Store points for drawing projectile path

    Vector2 CalculateProjectileTrajectory(Vector2 launch_position, Vector2 launch_velocity, const std::vector<Platform>& platforms);
    bool CheckTrajectoryPlatformCollision(Vector2 start_pos, Vector2 end_pos, const Platform& platform);
    Vector2 FindTrajectoryCollisionPoint(Vector2 start_pos, Vector2 end_pos, const Platform& platform);
    Vector2 CalculatePostLandingTrajectory(Vector2 landing_position, Vector2 landing_velocity, const Platform& landing_platform);

    void Update(float dt, const std::vector<Platform>& platforms = {});
    void Draw() override;
    void CheckCollision();
    // REMOVED: CheckPlatformCollisionSAT - now using only TwoLine collision detection
    void CheckPlatformCollisionTwoLine(const std::vector<Platform>& platforms);
    bool CheckAndHandleTransition(const std::vector<Platform>& platforms);
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
    
    // New reference frame-based trajectory calculation
    void CalculateMultiReferenceFrameTrajectory(const std::vector<Platform>& platforms);
    
    // Helper methods for reference frame approach
    struct PlatformPair {
        const Platform* slope_platform;        // First platform in the pair (any steepness)
        const Platform* horizontal_platform;   // Second platform in the pair (any steepness)
        Vector2 transition_point;
        bool has_intersection;
    };
    
    std::vector<PlatformPair> FindConnectedPlatformPairs(const std::vector<Platform>& platforms, Vector2 start_position);
    bool CalculateSingleReferenceFrame(const PlatformPair& pair, Vector2 start_pos, float initial_velocity, Vector2& end_pos, float& final_velocity);
    bool CanTransitionToNextPlatform(Vector2 position, float velocity, const Platform* current_platform, const Platform* next_platform);
    void DrawTwoLineCollisionDebug(const std::vector<Platform>& platforms);

    Rectangle inline Rect() const {
        return (Rectangle){ position.x, position.y, size.x, size.y };
    }
    void CheckPlatformCollision(Rectangle platform_rect);
    
};