#pragma once
#include "raylib.h"

struct Kinematic {
    Vector2 position = { 400.0f, 300.0f };
    Vector2 velocity = { 0.0f, 0.0f };
    float   orientation = 0.0f;              // radians, 0 = facing right
    float   rotation = 0.0f;              // angular velocity, rad/s
};

struct SteeringOutput {
    Vector2 linearAcceleration = { 0.0f, 0.0f };        // acceleration, px/s^2
    float   angularAcceleration = 0.0f;                  // angular acceleration, rad/s^2
};


class Agent {
public:
    Kinematic kinematic;
    float maxSpeed = 200.0f;
    float maxAcceleration = 400.0f;
	bool orientRotationToMovement = true; // if true, agent will rotate to face the direction of movement
    float minFacingSpeed = 1.0f; // minimum speen necesarry for turning
	float maxRotationSpeed = 10.0f; // maximum rotation speed in rad/s

    Agent() = default;
    explicit Agent(Vector2 startPosition);

    void    Update(const SteeringOutput& steering, float dt);
    void    Draw() const;
    Vector2 Forward() const;   // unit vector the agent is facing
	void	OrientRotationToMovement(float delta); // update orientation to face the direction of movement
};