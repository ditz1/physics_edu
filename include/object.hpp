#pragma once
#include "collision_utils.hpp"
#define gravity 9.81f


typedef class Object {
public:
    Vector2 position{0.0f, 0.0f};
    Vector2 velocity{0.0f, 0.0f};
    Vector2 acceleration{0.0f, 0.0f};
    float mass = 1.0f;
    bool is_grabbed = false;
    Vector2 grab_offset{0.0f, 0.0f};
    Vector2 grab_position{0.0f, 0.0f};

    std::vector<Vector2> path;

    void CheckGrab();
    void Grab(Vector2 mouse_position);
    virtual void Update(float dt);
    virtual void Draw();
    void ApplyGravity(float dt);
    virtual void DrawVectors();
} Object;