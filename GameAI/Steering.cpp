#include "Steering.h"
#include "raymath.h"

namespace Steering {

    SteeringOutput Seek(const Agent& agent, Vector2 target)
    {
        SteeringOutput out;

        // Direction from the agent to the target
        Vector2 toTarget = Vector2Subtract(target, agent.kinematic.position);

        // Guard against zero length, normalize, and request full acceleration that way
        if (Vector2Length(toTarget) > 0.0001f)
            out.linearAcceleration = Vector2Scale(Vector2Normalize(toTarget), agent.maxAcceleration);
        

        // angular stays 0: seek doesn't care about rotation
        return out;
    }

    SteeringOutput Flee(const Agent& agent, Vector2 target)
    {
        SteeringOutput out;

		// Flee is just the opposite of Seek: go away from the target
        Vector2 away = Vector2Subtract(agent.kinematic.position, target);

		// Guard against zero length, normalize, and request full acceleration that way
        if (Vector2Length(away) > 0.0001f)
            out.linearAcceleration = Vector2Scale(Vector2Normalize(away), agent.maxAcceleration);

        return out;
    }

    SteeringOutput Forward(const Agent& agent)
    {
        SteeringOutput out;
        out.linearAcceleration = Vector2Scale(agent.Forward(), agent.maxAcceleration);
        return out;
    }

    SteeringOutput Brake(const Agent& agent)
    {
        SteeringOutput out;
        Vector2 a = Vector2Scale(agent.kinematic.velocity, -4.0f);
        if (Vector2Length(a) > agent.maxAcceleration)
            a = Vector2Scale(Vector2Normalize(a), agent.maxAcceleration);
        out.linearAcceleration = a;
        return out;
    }

    SteeringOutput Arrive(const Agent& agent, Vector2 target,
        float targetRadius, float slowRadius,
        float timeToTarget)
    {
        SteeringOutput out;

        Vector2 toTarget = Vector2Subtract(target, agent.kinematic.position);
        float distance = Vector2Length(toTarget);

        // Desired speed depends on distance
        float desiredSpeed;
        if (distance < targetRadius)      desiredSpeed = 0.0f;
        else if (distance > slowRadius)   desiredSpeed = agent.maxSpeed;
        else                              desiredSpeed = agent.maxSpeed * distance / slowRadius;

        // Desired velocity: toward the target at that speed
        Vector2 desiredVelocity = { 0.0f, 0.0f };
        if (distance > 0.0001f)
            desiredVelocity = Vector2Scale(Vector2Normalize(toTarget), desiredSpeed);

        // Acceleration needed to reach it in timeToTarget seconds
        Vector2 accel = Vector2Scale(
            Vector2Subtract(desiredVelocity, agent.kinematic.velocity),
            1.0f / timeToTarget);

        // Clamp
        if (Vector2Length(accel) > agent.maxAcceleration)
            accel = Vector2Scale(Vector2Normalize(accel), agent.maxAcceleration);

        out.linearAcceleration = accel;
        return out;
    }

} // namespace Steering