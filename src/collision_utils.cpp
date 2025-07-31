#include "collision_utils.hpp"
#include "platform.hpp"
#include <algorithm>
#include <limits>
#include <cfloat>

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

bool CollisionUtils::LineIntersection(Vector2 line1Start, Vector2 line1End, 
                                     Vector2 line2Start, Vector2 line2End, 
                                     Vector2& intersection) {
    float x1 = line1Start.x, y1 = line1Start.y;
    float x2 = line1End.x, y2 = line1End.y;
    float x3 = line2Start.x, y3 = line2Start.y;
    float x4 = line2End.x, y4 = line2End.y;
    
    float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (fabs(denom) < 0.0001f) return false; // Lines are parallel
    
    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
    
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        intersection.x = x1 + t * (x2 - x1);
        intersection.y = y1 + t * (y2 - y1);
        return true;
    }
    
    return false;
}

RaycastHit CollisionUtils::RaycastToPlatform(Vector2 rayStart, Vector2 rayEnd, const Platform& platform) {
    RaycastHit hit = {false, {0, 0}, {0, 0}, 0, nullptr};
    
    // Get platform corners
    float cos_rot = cosf(platform.rotation * DEG2RAD);
    float sin_rot = sinf(platform.rotation * DEG2RAD);
    
    Vector2 halfSize = {platform.size.x * 0.5f, platform.size.y * 0.5f};
    
    // Get all four edges of the platform
    Vector2 corners[4];
    Vector2 localCorners[4] = {
        {-halfSize.x, -halfSize.y}, // top-left
        { halfSize.x, -halfSize.y}, // top-right  
        { halfSize.x,  halfSize.y}, // bottom-right
        {-halfSize.x,  halfSize.y}  // bottom-left
    };
    
    // Transform to world coordinates
    for (int i = 0; i < 4; i++) {
        corners[i].x = localCorners[i].x * cos_rot - localCorners[i].y * sin_rot + platform.position.x;
        corners[i].y = localCorners[i].x * sin_rot + localCorners[i].y * cos_rot + platform.position.y;
    }
    
    float closestDistance = FLT_MAX;
    Vector2 closestPoint;
    Vector2 closestNormal;
    
    // Check intersection with all four edges
    for (int i = 0; i < 4; i++) {
        Vector2 edgeStart = corners[i];
        Vector2 edgeEnd = corners[(i + 1) % 4];
        Vector2 intersection;
        
        if (LineIntersection(rayStart, rayEnd, edgeStart, edgeEnd, intersection)) {
            float distance = Vector2Distance(rayStart, intersection);
            if (distance < closestDistance) {
                closestDistance = distance;
                closestPoint = intersection;
                
                // Calculate edge normal (pointing outward from platform)
                Vector2 edge = Vector2Subtract(edgeEnd, edgeStart);
                closestNormal = Vector2Normalize((Vector2){-edge.y, edge.x});
                
                // Make sure normal points away from platform center
                Vector2 toPlatformCenter = Vector2Subtract(platform.position, intersection);
                if (Vector2DotProduct(closestNormal, toPlatformCenter) > 0) {
                    closestNormal = Vector2Scale(closestNormal, -1);
                }
            }
        }
    }
    
    if (closestDistance < FLT_MAX) {
        hit.hit = true;
        hit.point = closestPoint;
        hit.normal = closestNormal;
        hit.distance = closestDistance;
        hit.platform = const_cast<Platform*>(&platform);
    }
    
    return hit;
}

std::vector<RaycastHit> CollisionUtils::RaycastToAllPlatforms(Vector2 rayStart, Vector2 rayEnd, 
                                                             const std::vector<Platform>& platforms) {
    std::vector<RaycastHit> hits;
    
    for (const auto& platform : platforms) {
        RaycastHit hit = RaycastToPlatform(rayStart, rayEnd, platform);
        if (hit.hit) {
            hits.push_back(hit);
        }
    }
    
    // Sort by distance (closest first)
    std::sort(hits.begin(), hits.end(), [](const RaycastHit& a, const RaycastHit& b) {
        return a.distance < b.distance;
    });
    
    return hits;
}