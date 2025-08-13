#include "gorilla.hpp"
#include "box.hpp"
#include <iostream>

Gorilla::Gorilla() {
    size = {100.0f, 100.0f}; // default collision size
    position = {0.0f, 0.0f};
    texture = nullptr;
    UpdateCollisionZone();
}

Gorilla::Gorilla(Vector2 pos) {
    size = {100.0f, 100.0f};
    position = pos;
    texture = nullptr;
    UpdateCollisionZone();
}

Gorilla::~Gorilla() {
    // Texture is managed elsewhere, don't unload here
}

void Gorilla::Update(float dt) {
    UpdateCollisionZone();
    // Gorilla is mostly static, but we update collision zone in case it was moved
}

void Gorilla::Draw() {
    // Draw collision zone outline (always visible for goal indication)
    //DrawRectangleLinesEx(collision_zone, 1.0f, GREEN);
    
    // Draw the gorilla texture
    if (texture) {
        Vector2 texture_pos = {
            collision_zone.x,
            collision_zone.y
        };
        DrawTextureEx(*texture, texture_pos, 0.0f, scale, WHITE);
    }
    
    // Draw edit mode indicators (same as platforms)
    if (edit_mode) {
        DrawCircleV(position, 20.0f, RED);
        DrawText("GORILLA", (int)(position.x - 30), (int)(position.y - 40), 16, GREEN);
    }
}

bool Gorilla::CheckCollisionWithBox(const Box& box) {
    return CheckCollisionRecs(collision_zone, box.Rect());
}

void Gorilla::UpdateCollisionZone() {
    collision_zone = {
        position.x - size.x * 0.5f,
        position.y - size.y * 0.5f,
        size.x,
        size.y
    };
}