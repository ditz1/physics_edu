#pragma once
#include <object.hpp>

class Box : public Object {
public:
    Box();
    Box(int w, int h);
    ~Box();

    Vector2 size; // size of the box (width, height)
    Color color = RED;
    float mu_kinetic = 0.5f; // kinetic friction, 0.1 low friction, 1.0 high friction
    bool is_colliding = false; // used to check if the box is colliding with something

    void Update(float dt) override;
    void Draw() override;
    void CheckCollision();
    void ApplyFriction(float dt);
    Rectangle inline Rect() const {
        return (Rectangle){ position.x, position.y, size.x, size.y };
    }
    void CheckPlatformCollision(Rectangle platform_rect);
};