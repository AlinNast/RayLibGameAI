#pragma once
#include "raylib.h"
#include "Agent.h"

namespace Steering {
    SteeringOutput Seek(const Agent& agent, Vector2 target);
    SteeringOutput Flee(const Agent& agent, Vector2 target);
    SteeringOutput Forward(const Agent& agent);
    SteeringOutput Brake(const Agent& agent);
    SteeringOutput Arrive(const Agent& agent, Vector2 target,
        float targetRadius, float slowRadius,
        float timeToTarget = 0.1f);
}