#include "platform.hpp"


void Platform::Draw() {
    // position is now the center, so draw with center origin
    DrawRectanglePro(
        (Rectangle){ position.x, position.y, size.x, size.y }, 
        (Vector2){ size.x * 0.5f, size.y * 0.5f }, // origin at center
        rotation, 
        BEIGE
    );
}