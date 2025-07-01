#include "platform.hpp"


void Platform::Draw() {
    // Draw the platform as a rectangle
    DrawRectangleV(position, size, GRAY);
    
    // Optionally, draw the outline for better visibility
    DrawRectangleLinesEx((Rectangle){ position.x, position.y, size.x, size.y }, 2.0f, DARKGRAY);
}