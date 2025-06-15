#include "object.hpp"


class BallAndString : public Object {
public:
    Vector2 anchor;
    float angle;
    float angularSpeed;
    float radius;
    bool is_broken = false;
    
    BallAndString(Vector2 anchorPos, float rad, float initialAngle) {
        anchor = anchorPos;
        radius = rad;
        angle = initialAngle;
        angularSpeed = 0.05f; // Add this! Set initial spinning speed
        position = Position(); // Initialize position
    }
    
    void Update(float dt) override;
    Vector2 Position();
    void Draw() override;
    void DrawVectors() override;
    void Break();
    void Reset();
};