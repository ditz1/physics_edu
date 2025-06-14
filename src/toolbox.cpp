#include <toolbox.hpp>


void Toolbox::Draw() {
    // draw bar on top
    float bar_height = 150.0f;
    float bar_width = GetScreenWidth();
    Rectangle bar_rect = { 10, 10, bar_width - 20, bar_height };
    DrawRectangleRounded(bar_rect, 0.3f, 20, LIGHTGRAY);

    // draw selectable objects
    float button_width = 50.0f;
    float button_height = 50.0f;
    int button_start_x = 20;
    int button_start_y = 20;
    DrawRectangle(button_start_x, button_start_y, button_width, button_height, RED);
    DrawRectangle(button_start_x + button_width + 10, button_start_y, button_width, button_height, GREEN);
}