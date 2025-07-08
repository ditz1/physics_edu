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

    void Update(float dt) override;
    void Draw() override;
    void CheckCollision();
    void CheckPlatformCollisionSAT(const Platform& platform, int platform_id);    
    void ApplyFriction(float dt);
    Rectangle inline Rect() const {
        return (Rectangle){ position.x, position.y, size.x, size.y };
    }
    void CheckPlatformCollision(Rectangle platform_rect);
    
};