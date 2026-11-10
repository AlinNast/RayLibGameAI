#pragma once
#include "SteeringBehavior.h"
#include "Steering.h"
#include "raymath.h"
#include <cmath>
#include <functional>


class SeekBehavior : public SteeringBehavior {
public:
    explicit SeekBehavior(const Kinematic* target) : target(target) {}
    SteeringOutput GetSteering(const Agent& agent, const World&) override
    {
        return Steering::Seek(agent, target->position);
    }
    const Kinematic* target;
};


class FleeBehavior : public SteeringBehavior {
public:
    explicit FleeBehavior(const Kinematic* target) : target(target) {}
    SteeringOutput GetSteering(const Agent& agent, const World&) override
    {
        return Steering::Flee(agent, target->position);
    }
    const Kinematic* target;
};


class ArriveBehavior : public SteeringBehavior {
public:
    ArriveBehavior(const Kinematic* target, float targetRadius, float slowRadius)
        : target(target), targetRadius(targetRadius), slowRadius(slowRadius) {
    }
    SteeringOutput GetSteering(const Agent& agent, const World&) override
    {
        return Steering::Arrive(agent, target->position, targetRadius, slowRadius, timeToTarget);
    }
    const Kinematic* target;
    float targetRadius;
    float slowRadius;
    float timeToTarget = 0.1f;
};


class PursueBehavior : public SteeringBehavior {
public:
    explicit PursueBehavior(const Kinematic* target) : target(target) {}
    SteeringOutput GetSteering(const Agent& agent, const World&) override
    {
        return Steering::Pursue(agent, target->position, target->velocity, maxPrediction);
    }
    const Kinematic* target;
    float maxPrediction = 1.0f;
};


class EvadeBehavior : public SteeringBehavior {
public:
    explicit EvadeBehavior(const Kinematic* target) : target(target) {}
    SteeringOutput GetSteering(const Agent& agent, const World&) override
    {
        return Steering::Evade(agent, target->position, target->velocity, maxPrediction);
    }
    const Kinematic* target;
    float maxPrediction = 1.0f;
};


class WanderBehavior : public SteeringBehavior {
public:
    SteeringOutput GetSteering(const Agent& agent, const World& world) override
    {
        return Steering::Wander(agent, data, world.deltaTime);
    }
    Steering::WanderData data;   // the state now lives INSIDE the behavior
};


class BrakeBehavior : public SteeringBehavior {
public:
    SteeringOutput GetSteering(const Agent& agent, const World&) override
    {
        return Steering::Brake(agent);
    }
};


class SeparationBehavior : public SteeringBehavior {
public:
    /// <summary>
    /// Calculates a steering output that pushes the agent away from other agents
    /// </summary>
    /// <param name="threshold"> Minimum distance at which this activates</param>
    /// <param name="decay"> how strong the push is (" decay/dist^2 ") </param>
    SeparationBehavior(float threshold, float decay) : threshold(threshold), decay(decay) { }

    SteeringOutput GetSteering(const Agent& agent, const World& world) override // check with all the agents in the world
    {
        SteeringOutput out;

        for (const Agent* other : world.agents) {
            if (other == &agent) continue;                  // don't flee from yourself

            Vector2 away = Vector2Subtract(agent.kinematic.position, other->kinematic.position);
			float dist = Vector2Length(away);               // distance to the other agent
            if (dist >= threshold) continue;                // outside personal space

            if (dist < 0.0001f) {
                // Exactly on top of each other: there's no direction to flee in.
                // Both agents run this code, so they must pick OPPOSITE directions
                // or they would move together forever.
                away = std::less<const Agent*>{}(&agent, other) ? Vector2{ 1, 0 } : Vector2{ -1, 0 };
                dist = 0.0001f;
            }

            float strength = fminf(decay / (dist * dist), agent.maxAcceleration); //strenght is inverse ro distance, caped to max acceleration
            out.linearAcceleration = Vector2Add(out.linearAcceleration,
                Vector2Scale(Vector2Normalize(away), strength));
        }

        if (Vector2Length(out.linearAcceleration) > agent.maxAcceleration)
            out.linearAcceleration = Vector2Scale(Vector2Normalize(out.linearAcceleration), agent.maxAcceleration);

        return out;
    }

    float threshold;   // personal-space radius (px)
    float decay;       // push strength coefficient
};


class CollisionAvoidanceBehavior : public SteeringBehavior {
public:
    explicit CollisionAvoidanceBehavior(float maxLookAhead = 2.0f, float margin = 10.0f) : maxLookAhead(maxLookAhead), margin(margin) { }

    SteeringOutput GetSteering(const Agent& agent, const World& world) override
    {
        bool    found = false;
        float   shortestTime = maxLookAhead;   // ignore collisions further away than this
        Vector2 threatRelPos = { 0, 0 };
        Vector2 threatRelVel = { 0, 0 };

        for (const Agent* other : world.agents) {
            if (other == &agent) continue;

            Vector2 relPos = Vector2Subtract(other->kinematic.position, agent.kinematic.position);
            Vector2 relVel = Vector2Subtract(other->kinematic.velocity, agent.kinematic.velocity);
            float combined = agent.radius + other->radius + margin;
            float dist = Vector2Length(relPos);

            float t = 0.0f;                          // 0 means "already overlapping"
            if (dist >= combined) {
                float relSpeedSq = Vector2DotProduct(relVel, relVel);
                if (relSpeedSq < 0.0001f) continue;  // same velocity: the gap never changes

                t = -Vector2DotProduct(relPos, relVel) / relSpeedSq;
                if (t <= 0.0f) continue;             // moving apart

                Vector2 closest = Vector2Add(relPos, Vector2Scale(relVel, t));
                if (Vector2Length(closest) >= combined) continue;   // will pass safely
            }

            if (t >= shortestTime) continue;         // not the most urgent one
            found = true;
            shortestTime = t;
            threatRelPos = relPos;
            threatRelVel = relVel;
        }

        if (!found) return {};                       // nothing to dodge: say nothing

        // Where the threat will be relative to me at the moment of closest approach
        Vector2 avoidFrom = Vector2Add(threatRelPos, Vector2Scale(threatRelVel, shortestTime));

        if (shortestTime == 0.0f)
            avoidFrom = threatRelPos;                // already touching: flee from where it is now
        else if (Vector2Length(avoidFrom) < 0.0001f)
            avoidFrom = { -threatRelVel.y, threatRelVel.x };   // dead-centre head-on: sidestep

        SteeringOutput out;
        out.linearAcceleration = Vector2Scale(Vector2Normalize(avoidFrom), -agent.maxAcceleration);
        return out;
    }

    float maxLookAhead;   // seconds to look into the future
    float margin;         // extra personal space added to both radii (px)
};