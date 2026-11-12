#include "Agent.h"
#include "raymath.h"   // Vector2Add, Vector2Scale, ... come from here
#include "SteeringBehavior.h"
#include "World.h"
#include <cmath>


/// <summary>
/// Constructor for an agent, setting its initial position.
/// </summary>
/// <param name="startPosition"></param>
Agent::Agent(Vector2 startPosition)
{
    kinematic.position = startPosition;
}
Agent::Agent() = default;
Agent::~Agent() = default;


/// <summary>
/// Returns the forward direction of the agent (unit vector in the direction of its orientation).
/// </summary>
/// <returns>(unit vector in the direction of its orientation).</returns>
Vector2 Agent::Forward() const
{
	return { cosf(kinematic.orientation), sinf(kinematic.orientation) }; // orientation is an angle in radians, so direction is (cosA, sinA)
}


static float WrapAngle(float a)   // bring any angle into [-PI, PI]
{
    while (a > PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}


/// <summary>
/// Apply the steering to the agent's kinematic state, Orient the agent if needed, and wrap around the screen.
/// </summary>
/// <param name="steering"></param>
/// <param name="delta"></param>
void Agent::Update(const SteeringOutput& steering, float delta)
{
    // Move using the old velocity
    kinematic.position = Vector2Add(kinematic.position, Vector2Scale(kinematic.velocity, delta));  // P = V*dt + P

    // Apply the steering as acceleration
	kinematic.velocity = Vector2Add(kinematic.velocity, Vector2Scale(steering.linearAcceleration, delta)); // V = lAcc*dt + V 

    // Clamp speed
    if (Vector2Length(kinematic.velocity) > maxSpeed)
        kinematic.velocity = Vector2Scale(Vector2Normalize(kinematic.velocity), maxSpeed);

	// Update orientation based on rotation
    if (orientRotationToMovement) {
        OrientRotationToMovement(delta);
    }
	else {  // use the rotation from the steering output, not used anywhere
		kinematic.orientation += kinematic.rotation * delta;  // angle = angular velocity * dt
		kinematic.rotation += steering.angularAcceleration * delta;  // angular velocity = angular acceleration * dt
	}

    // Wrap around the screen so it doesn't disappear while testing
    float w = (float)GetScreenWidth();
    float h = (float)GetScreenHeight();
    if (kinematic.position.x < 0) kinematic.position.x += w;
    if (kinematic.position.x > w) kinematic.position.x -= w;
    if (kinematic.position.y < 0) kinematic.position.y += h;
    if (kinematic.position.y > h) kinematic.position.y -= h;
}


/// <summary>
/// Draw the agent as a triangle pointing in the direction of its orientation, size and angle are hardcoded
/// </summary>
void Agent::Draw(bool isSelected) const
{
    const float size = 25.0f;
    const float back = 140.0f * DEG2RAD;
    const Vector2 p = kinematic.position;
    const float   o = kinematic.orientation;

    Vector2 tip = { p.x + cosf(o) * size,        p.y + sinf(o) * size };
    Vector2 left = { p.x + cosf(o - back) * size, p.y + sinf(o - back) * size };
    Vector2 right = { p.x + cosf(o + back) * size, p.y + sinf(o + back) * size };

    DrawTriangle(tip, left, right, isSelected ? selectedColor : defaultColor);
}


/// <summary>
/// Orient the agent to face the direction of movement, Only active if orientRotationToMovement is true
/// </summary>
/// <param name="delta"></param>
void Agent::OrientRotationToMovement(float delta) {
    kinematic.rotation = 0.0f;   // Angular velocity is not used when orienting to movement, this resets it to be sure

	if (Vector2Length(kinematic.velocity) > minFacingSpeed) {   // only orient if moving fast enough
        float velocityAngle = atan2f(kinematic.velocity.y, kinematic.velocity.x);
        float difference = WrapAngle(velocityAngle - kinematic.orientation);

		// clamp the rotation to the maximum rotation speed
        float maxStep = maxRotationSpeed * delta;
        if (difference > maxStep) difference = maxStep;
        if (difference < -maxStep) difference = -maxStep;

        kinematic.orientation += difference;
    }

}

/// <summary>
/// Uses the agent's steering behaviour to compute the steering output for this agent and returns it. If the agent has no behavior, returns a zeroed SteeringOutput.
/// </summary>
/// <param name="world">a reference to the world is needed for the agent to know what is out there</param>
/// <returns></returns>
SteeringOutput Agent::ComputeSteering(const World& world) const
{
    if (!behavior) return {};                 // no brain: no steering
    return behavior->GetSteering(*this, world);
}