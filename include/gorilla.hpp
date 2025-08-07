#pragma once
#include "object.hpp"

class Gorilla : public Object {
public:
    Gorilla();
    Gorilla(Vector2 pos);
    ~Gorilla();

    Vector2 size; // collision area size
    Color color = YELLOW;
    Rectangle collision_zone;
    Texture2D* texture;
    float scale = 0.125f; // texture scale factor
    
    void Update(float dt) override;
    void Draw() override;
    bool CheckCollisionWithBox(const class Box& box);
    void UpdateCollisionZone();
    
    Rectangle inline Rect() const {
        return collision_zone;
    }
};