#include "Agent.h"
#include "raymath.h"   // Vector2Add, Vector2Scale, ...
#include <cmath>

Agent::Agent(Vector2 startPosition)
{
    kinematic.position = startPosition;
}

Vector2 Agent::Forward() const
{
    return { cosf(kinematic.orientation), sinf(kinematic.orientation) };
}

void Agent::Update(const SteeringOutput& steering, float delta)
{
    // Move using the current velocity
    kinematic.position = Vector2Add(kinematic.position, Vector2Scale(kinematic.velocity, delta));
    kinematic.orientation += kinematic.rotation * delta;

    // Apply the steering as acceleration
    kinematic.velocity = Vector2Add(kinematic.velocity, Vector2Scale(steering.linear, delta));
    kinematic.rotation += steering.angular * delta;

    // Clamp speed
    if (Vector2Length(kinematic.velocity) > maxSpeed)
        kinematic.velocity = Vector2Scale(Vector2Normalize(kinematic.velocity), maxSpeed);

    // Wrap around the screen so it doesn't disappear while testing
    float w = (float)GetScreenWidth();
    float h = (float)GetScreenHeight();
    if (kinematic.position.x < 0) kinematic.position.x += w;
    if (kinematic.position.x > w) kinematic.position.x -= w;
    if (kinematic.position.y < 0) kinematic.position.y += h;
    if (kinematic.position.y > h) kinematic.position.y -= h;
}

void Agent::Draw() const
{
    const float size = 25.0f;
    const float back = 140.0f * DEG2RAD;
    const Vector2 p = kinematic.position;
    const float   o = kinematic.orientation;

    Vector2 tip = { p.x + cosf(o) * size,        p.y + sinf(o) * size };
    Vector2 left = { p.x + cosf(o - back) * size, p.y + sinf(o - back) * size };
    Vector2 right = { p.x + cosf(o + back) * size, p.y + sinf(o + back) * size };

    DrawTriangle(tip, left, right, MAROON);
}