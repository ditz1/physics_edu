#include "collision_utils.hpp"
#include "platform.hpp"
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
            return info; 
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

// New two-line collision detection functions
std::vector<LineSegment> CollisionUtils::GetBoxBottomLines(const Vector2& boxPosition, const Vector2& boxSize, float boxRotation) {
    std::vector<LineSegment> lines;
    
    // Convert rotation to radians
    float rotationRad = boxRotation * DEG2RAD;
    float cos_rot = cosf(rotationRad);
    float sin_rot = sinf(rotationRad);
    
    // Calculate the bottom corners of the box (local coordinates)
    Vector2 localBottomLeft = { -boxSize.x * 0.5f, boxSize.y * 0.5f };
    Vector2 localBottomRight = { boxSize.x * 0.5f, boxSize.y * 0.5f };
    
    // Rotate the bottom corners
    Vector2 rotatedBottomLeft = {
        localBottomLeft.x * cos_rot - localBottomLeft.y * sin_rot + boxPosition.x,
        localBottomLeft.x * sin_rot + localBottomLeft.y * cos_rot + boxPosition.y
    };
    
    Vector2 rotatedBottomRight = {
        localBottomRight.x * cos_rot - localBottomRight.y * sin_rot + boxPosition.x,
        localBottomRight.x * sin_rot + localBottomRight.y * cos_rot + boxPosition.y
    };
    
    // Create two vertical lines extending downward from the bottom corners
    // The lines will be drawn to the nearest platform surface
    Vector2 line1Start = rotatedBottomLeft;
    Vector2 line1End = { rotatedBottomLeft.x, rotatedBottomLeft.y + 1000.0f }; // Temporary end point
    
    Vector2 line2Start = rotatedBottomRight;
    Vector2 line2End = { rotatedBottomRight.x, rotatedBottomRight.y + 1000.0f }; // Temporary end point
    
    lines.push_back({ line1Start, line1End });
    lines.push_back({ line2Start, line2End });
    
    return lines;
}

TwoLineCollisionInfo CollisionUtils::CheckTwoLineCollision(const std::vector<LineSegment>& lines, const std::vector<Platform>& platforms) {
    TwoLineCollisionInfo collisionInfo = { false, {0, 0}, std::numeric_limits<float>::max(), {0, 1}, -1 };
    
    // Check each line against each platform
    for (const auto& line : lines) {
        for (size_t i = 0; i < platforms.size(); i++) {
            const auto& platform = platforms[i];
            
            float distance = GetDistanceToPlatform(line, platform);
            
            // If this line is closer to a platform than any previous collision
            if (distance < collisionInfo.distanceToPlatform) {
                Vector2 closestPoint = GetClosestPointOnPlatform(line, platform);
                
                collisionInfo.hasCollision = true;
                collisionInfo.collisionPoint = closestPoint;
                collisionInfo.distanceToPlatform = distance;
                collisionInfo.platformId = static_cast<int>(i);
                
                // Calculate normal (pointing upward from platform)
                Vector2 platformDirection = Vector2Subtract(platform.top_right, platform.top_left);
                Vector2 normal = { -platformDirection.y, platformDirection.x };
                normal = Vector2Normalize(normal);
                
                // Ensure normal points upward
                if (normal.y > 0) {
                    normal = Vector2Scale(normal, -1);
                }
                
                collisionInfo.normal = normal;
            }
        }
    }
    
    // If no collision found, reset the collision info
    if (collisionInfo.distanceToPlatform == std::numeric_limits<float>::max()) {
        collisionInfo.hasCollision = false;
        collisionInfo.platformId = -1;
    }
    
    return collisionInfo;
}

float CollisionUtils::GetDistanceToPlatform(const LineSegment& line, const Platform& platform) {
    // Get the platform's top edge as a line segment
    LineSegment platformLine = { platform.top_left, platform.top_right };
    
    // Find the closest point on the platform line to our vertical line
    Vector2 closestPoint = GetClosestPointOnPlatform(line, platform);
    
    // Calculate the actual distance from line start to the platform surface
    float distance = Vector2Distance(line.start, closestPoint);
    
    // Only consider collision if the line is above the platform surface
    // and the distance is very small (essentially touching)
    if (line.start.y > closestPoint.y && distance < 2.0f) {
        return distance;
    }
    
    return std::numeric_limits<float>::max(); // No collision
}

Vector2 CollisionUtils::GetClosestPointOnPlatform(const LineSegment& line, const Platform& platform) {
    // Get the platform's top edge as a line segment
    LineSegment platformLine = { platform.top_left, platform.top_right };
    
    // Get the X coordinate of the vertical line
    float lineX = line.start.x;
    
    // Find the Y coordinate on the platform line for this X coordinate
    Vector2 platformDir = Vector2Subtract(platformLine.end, platformLine.start);
    float platformLength = Vector2Length(platformDir);
    
    if (platformLength == 0) {
        return platformLine.start; // Platform is a point
    }
    
    // Calculate the Y coordinate on the platform line for the given X coordinate
    // This is the point where a vertical line at lineX would intersect the platform
    float platformSlope = platformDir.y / platformDir.x;
    float platformY = platformLine.start.y + platformSlope * (lineX - platformLine.start.x);
    
    // Clamp to platform bounds
    float minX = fmin(platformLine.start.x, platformLine.end.x);
    float maxX = fmax(platformLine.start.x, platformLine.end.x);
    
    if (lineX < minX || lineX > maxX) {
        // Line is outside platform bounds, return closest endpoint
        if (lineX < minX) {
            return platformLine.start;
        } else {
            return platformLine.end;
        }
    }
    
    // Return the point on the platform line at the given X coordinate
    return {lineX, platformY};
}