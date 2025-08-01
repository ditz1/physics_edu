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

class CollisionUtils {
public:
    static std::vector<Vector2> GetRectangleCorners(const RotatedRectangle& rect);
    static std::vector<Vector2> GetNormals(const std::vector<Vector2>& corners);
    static void ProjectOntoAxis(const std::vector<Vector2>& corners, Vector2 axis, float& min, float& max);
    static bool CheckSATCollision(const RotatedRectangle& rect1, const RotatedRectangle& rect2);
    static CollisionInfo GetCollisionInfo(const RotatedRectangle& rect1, const RotatedRectangle& rect2);
};