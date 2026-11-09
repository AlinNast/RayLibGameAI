#include "Steering.h"
#include "raymath.h"
#include <cmath>

namespace Steering {
    // Random value in [-1, 1], biased toward 0 (the book's "randomBinomial")
    static float RandomBinomial()
    {
        float a = GetRandomValue(0, 1000) / 1000.0f;
        float b = GetRandomValue(0, 1000) / 1000.0f;
        return a - b;
    }

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

    SteeringOutput Arrive(const Agent& agent, Vector2 target, float targetRadius, float slowRadius, float timeToTarget)
    {
        SteeringOutput out;

		// Get the direction and distance to the target
        Vector2 toTarget = Vector2Subtract(target, agent.kinematic.position);
        float distance = Vector2Length(toTarget);

        // Desired speed depends on distance
        float desiredSpeed;
		if (distance < targetRadius)      desiredSpeed = 0.0f; // if its inside the target radius, stop
		else if (distance > slowRadius)   desiredSpeed = agent.maxSpeed; // if its outside the slow radius, go max speed
		else                              desiredSpeed = agent.maxSpeed * distance / slowRadius; // otherwise go at a speed proportionate to the distance

		// Desired velocity scales with the desired speed and points in the direction of the target
        Vector2 desiredVelocity = { 0.0f, 0.0f };
        if (distance > 0.0001f)
            desiredVelocity = Vector2Scale(Vector2Normalize(toTarget), desiredSpeed);

        // Acceleration needed to reach it in timeToTarget seconds
        Vector2 accel = Vector2Scale(Vector2Subtract(desiredVelocity, agent.kinematic.velocity),1.0f / timeToTarget);

        // Clamp
        if (Vector2Length(accel) > agent.maxAcceleration)
            accel = Vector2Scale(Vector2Normalize(accel), agent.maxAcceleration);

        out.linearAcceleration = accel;
        return out;
    }

    SteeringOutput Pursue(const Agent& agent, Vector2 targetPos, Vector2 targetVel, float maxPrediction)
    {
        Vector2 toTarget = Vector2Subtract(targetPos, agent.kinematic.position);
        float distance = Vector2Length(toTarget);

        // How fast the agent is currently moving
        float speed = Vector2Length(agent.kinematic.velocity);

        // Time to reach the target if it stood still, capped
        float prediction;
        if (speed <= distance / maxPrediction)
            prediction = maxPrediction;
        else
            prediction = distance / speed;

        // Where the target will be after that time
        Vector2 futurePosition = Vector2Add(targetPos, Vector2Scale(targetVel, prediction));

        // Seek the predicted point
        return Seek(agent, futurePosition);
    }

    SteeringOutput Evade(const Agent& agent, Vector2 targetPos, Vector2 targetVel, float maxPrediction)
    {
        Vector2 toTarget = Vector2Subtract(targetPos, agent.kinematic.position);
        float distance = Vector2Length(toTarget);

        // How fast the agent is currently moving
        float speed = Vector2Length(agent.kinematic.velocity);

        // Time to reach the target if it stood still, capped
        float prediction;
        if (speed <= distance / maxPrediction)
            prediction = maxPrediction;
        else
            prediction = distance / speed;

        // Where the target will be after that time
        Vector2 futurePosition = Vector2Add(targetPos, Vector2Scale(targetVel, prediction));

        // Seek the predicted point
        return Flee(agent, futurePosition);
    }

    SteeringOutput Wander(const Agent& agent, WanderData& data, float delta)
    {
        // Nudge the target angle by a small random amount
        data.angle += RandomBinomial() * data.rate * delta;

        // Put the circle's center a fixed distance in front of the agent
        data.circleCenter = Vector2Add(agent.kinematic.position,
            Vector2Scale(agent.Forward(), data.offset));

        // Find the point on the circle: the angle is relative to the agent's facing
        float worldAngle = agent.kinematic.orientation + data.angle;
        Vector2 onCircle = { cosf(worldAngle) * data.radius,
                             sinf(worldAngle) * data.radius };
        data.target = Vector2Add(data.circleCenter, onCircle);

        // Seek that point
        return Seek(agent, data.target);
    }

} // namespace Steering