#pragma once
#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

// Forward declaration
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

struct RaycastHit {
    bool hit;
    Vector2 point;
    Vector2 normal;
    float distance;
    Platform* platform; // Which platform was hit
};

class CollisionUtils {
public:
    static std::vector<Vector2> GetRectangleCorners(const RotatedRectangle& rect);
    static std::vector<Vector2> GetNormals(const std::vector<Vector2>& corners);
    static void ProjectOntoAxis(const std::vector<Vector2>& corners, Vector2 axis, float& min, float& max);
    static bool CheckSATCollision(const RotatedRectangle& rect1, const RotatedRectangle& rect2);
    static CollisionInfo GetCollisionInfo(const RotatedRectangle& rect1, const RotatedRectangle& rect2);
    
    // New raycast functions
    static bool LineIntersection(Vector2 line1Start, Vector2 line1End, 
                                Vector2 line2Start, Vector2 line2End, 
                                Vector2& intersection);
    static RaycastHit RaycastToPlatform(Vector2 rayStart, Vector2 rayEnd, const Platform& platform);
    static std::vector<RaycastHit> RaycastToAllPlatforms(Vector2 rayStart, Vector2 rayEnd, 
                                                         const std::vector<Platform>& platforms);
};