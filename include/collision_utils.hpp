#pragma once
#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>

class Platform;

struct RotatedRectangle {
    Vector2 center;
    Vector2 size;
    float rotation; // in radians
};

struct CollisionInfo {
    bool hasCollision;
    Vector2 normal;
    float penetration;
};

// New structures for two-line collision system
struct LineSegment {
    Vector2 start;
    Vector2 end;
};

struct TwoLineCollisionInfo {
    bool hasCollision;
    Vector2 collisionPoint;
    float distanceToPlatform;
    Vector2 normal;
    int platformId;
};

class CollisionUtils {
public:
    static std::vector<Vector2> GetRectangleCorners(const RotatedRectangle& rect);
    static std::vector<Vector2> GetNormals(const std::vector<Vector2>& corners);
    static void ProjectOntoAxis(const std::vector<Vector2>& corners, Vector2 axis, float& min, float& max);
    static bool CheckSATCollision(const RotatedRectangle& rect1, const RotatedRectangle& rect2);
    static CollisionInfo GetCollisionInfo(const RotatedRectangle& rect1, const RotatedRectangle& rect2);
    
    // New two-line collision detection functions
    static std::vector<LineSegment> GetBoxBottomLines(const Vector2& boxPosition, const Vector2& boxSize, float boxRotation);
    static TwoLineCollisionInfo CheckTwoLineCollision(const std::vector<LineSegment>& lines, const std::vector<Platform>& platforms);
    static float GetDistanceToPlatform(const LineSegment& line, const Platform& platform);
    static Vector2 GetClosestPointOnPlatform(const LineSegment& line, const Platform& platform);
};