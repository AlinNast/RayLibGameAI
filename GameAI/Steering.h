#pragma once
#include "raylib.h"
#include "Agent.h"

namespace Steering {

    struct WanderData {
        // Settings
        float offset = 150.0f;   // distance of the circle ahead of the agent (px)
        float radius = 50.0f;    // circle radius (px), keep it smaller than offset
        float rate = 8.0f;     // max change of the target angle (rad/s)

        // State, which persists between frames
        float angle = 0.0f;      // target angle relative to the agent's facing

        // Output, for debug drawing only
        Vector2 circleCenter = { 0.0f, 0.0f };
        Vector2 target = { 0.0f, 0.0f };
    };


    SteeringOutput Seek(const Agent& agent, Vector2 target);
    SteeringOutput Flee(const Agent& agent, Vector2 target);
    SteeringOutput Forward(const Agent& agent);
    SteeringOutput Brake(const Agent& agent);
    SteeringOutput Arrive(const Agent& agent, Vector2 target,
        float targetRadius, float slowRadius,
        float timeToTarget = 0.1f);
    SteeringOutput Pursue(const Agent& agent, Vector2 targetPos, Vector2 targetVel,
        float maxPrediction = 1.0f);
    SteeringOutput Evade(const Agent& agent, Vector2 targetPos, Vector2 targetVel,
        float maxPrediction = 1.0f);
    SteeringOutput Wander(const Agent& agent, WanderData& data, float delta);
}