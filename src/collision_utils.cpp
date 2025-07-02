#include "collision_utils.hpp"
#include <algorithm>
#include <limits>

std::vector<Vector2> CollisionUtils::GetRectangleCorners(const RotatedRectangle& rect) {
    Vector2 halfSize = { rect.size.x * 0.5f, rect.size.y * 0.5f };
    
    // Local corners (relative to center)
    std::vector<Vector2> localCorners = {
        { -halfSize.x, -halfSize.y }, // Top-left
        {  halfSize.x, -halfSize.y }, // Top-right
        {  halfSize.x,  halfSize.y }, // Bottom-right
        { -halfSize.x,  halfSize.y }  // Bottom-left
    };
    
    std::vector<Vector2> worldCorners;
    float cos_rot = cosf(rect.rotation);
    float sin_rot = sinf(rect.rotation);
    
    for (const auto& corner : localCorners) {
        Vector2 rotated = {
            corner.x * cos_rot - corner.y * sin_rot,
            corner.x * sin_rot + corner.y * cos_rot
        };
        worldCorners.push_back(Vector2Add(rect.center, rotated));
    }
    
    return worldCorners;
}

std::vector<Vector2> CollisionUtils::GetNormals(const std::vector<Vector2>& corners) {
    std::vector<Vector2> normals;
    
    for (size_t i = 0; i < corners.size(); i++) {
        Vector2 edge = Vector2Subtract(corners[(i + 1) % corners.size()], corners[i]);
        Vector2 normal = { -edge.y, edge.x }; // Perpendicular to edge
        normal = Vector2Normalize(normal);
        normals.push_back(normal);
    }
    
    return normals;
}

void CollisionUtils::ProjectOntoAxis(const std::vector<Vector2>& corners, Vector2 axis, float& min, float& max) {
    min = std::numeric_limits<float>::max();
    max = std::numeric_limits<float>::lowest();
    
    for (const auto& corner : corners) {
        float projection = Vector2DotProduct(corner, axis);
        min = std::min(min, projection);
        max = std::max(max, projection);
    }
}

bool CollisionUtils::CheckSATCollision(const RotatedRectangle& rect1, const RotatedRectangle& rect2) {
    auto corners1 = GetRectangleCorners(rect1);
    auto corners2 = GetRectangleCorners(rect2);
    
    auto normals1 = GetNormals(corners1);
    auto normals2 = GetNormals(corners2);
    
    // Check all normals from both rectangles
    std::vector<Vector2> allNormals;
    allNormals.insert(allNormals.end(), normals1.begin(), normals1.end());
    allNormals.insert(allNormals.end(), normals2.begin(), normals2.end());
    
    for (const auto& axis : allNormals) {
        float min1, max1, min2, max2;
        ProjectOntoAxis(corners1, axis, min1, max1);
        ProjectOntoAxis(corners2, axis, min2, max2);
        
        // Check for separation
        if (max1 < min2 || max2 < min1) {
            return false; // Separating axis found, no collision
        }
    }
    
    return true; // No separating axis found, collision detected
}

// New function to get collision info with penetration depth
CollisionInfo CollisionUtils::GetCollisionInfo(const RotatedRectangle& rect1, const RotatedRectangle& rect2) {
    CollisionInfo info = { false, {0, 0}, 0 };
    
    auto corners1 = GetRectangleCorners(rect1);
    auto corners2 = GetRectangleCorners(rect2);
    
    auto normals1 = GetNormals(corners1);
    auto normals2 = GetNormals(corners2);
    
    std::vector<Vector2> allNormals;
    allNormals.insert(allNormals.end(), normals1.begin(), normals1.end());
    allNormals.insert(allNormals.end(), normals2.begin(), normals2.end());
    
    float minOverlap = std::numeric_limits<float>::max();
    Vector2 collisionNormal = {0, 0};
    
    for (const auto& axis : allNormals) {
        float min1, max1, min2, max2;
        ProjectOntoAxis(corners1, axis, min1, max1);
        ProjectOntoAxis(corners2, axis, min2, max2);
        
        // Check for separation
        if (max1 < min2 || max2 < min1) {
            return info; // No collision
        }
        
        float overlap = std::min(max1, max2) - std::max(min1, min2);
        if (overlap < minOverlap) {
            minOverlap = overlap;
            collisionNormal = axis;
        }
    }
    
    // Ensure normal points from rect1 to rect2
    Vector2 centerDiff = Vector2Subtract(rect2.center, rect1.center);
    if (Vector2DotProduct(collisionNormal, centerDiff) < 0) {
        collisionNormal = Vector2Scale(collisionNormal, -1);
    }
    
    info.hasCollision = true;
    info.normal = collisionNormal;
    info.penetration = minOverlap;
    
    return info;
}