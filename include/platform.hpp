#pragma once
#include "object.hpp" 

// platforms are static objects that can be used as surfaces for other objects to interact with
// they are collidable, but should never move
class Platform : public Object {
public:
    Platform(Vector2 position, Vector2 size) {
        this->position = position;
        this->size = size;
        this->mass = 1.0f; // Default mass
        this->is_grabbed = false; // Platforms are not grabbed by default
    }
    Vector2 size;

    void Draw() override;

    Rectangle inline Rect() const {
        return (Rectangle){ position.x, position.y, size.x, size.y };
    }

    void Update(float dt) override {
        // Platforms typically do not move, so no update logic needed
    }


};