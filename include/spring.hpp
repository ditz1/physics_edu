#pragma once
#include "object.hpp"


class Spring : public Object {
public:
    Spring();
    ~Spring();
    Vector2 anchor;
    float mass;
    float spring_constant;
    float damping_factor;
    float rest_length;
    Vector2 _force; // should never be used directly, only for debugging
    float _compression;
    float radius = 30.0f; // purely for drawing

    void Update(float dt) override;
    void Draw() override;
};