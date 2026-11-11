#pragma once
#include "SteeringBehavior.h"
#include "Steering.h"
#include "raymath.h"
#include "Collision.h"
#include <cmath>
#include <functional>
#include <array>
#include "Path.h"
#include "Formation.h"


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

class ObstacleAvoidanceBehavior : public SteeringBehavior {
public:
    ObstacleAvoidanceBehavior(float lookAhead = 120.0f, float whiskerAngleDeg = 35.0f)
        : lookAhead(lookAhead), whiskerAngle(whiskerAngleDeg* DEG2RAD) {
    }

    SteeringOutput GetSteering(const Agent& agent, const World& world) override
    {
        steering = false;
        const Vector2 pos = agent.kinematic.position;

        // Heading: along the velocity, or the facing if almost stationary
        Vector2 dir = (Vector2Length(agent.kinematic.velocity) > 1.0f)
            ? Vector2Normalize(agent.kinematic.velocity)
            : agent.Forward();
        const Vector2 right = Vector2Rotate(dir, PI / 2.0f);   // the agent's right-hand side

        struct RayDef { Vector2 dir; float length; };
        const RayDef defs[3] = {
            { dir,                                lookAhead },         // 0: center
            { Vector2Rotate(dir,  whiskerAngle),  lookAhead * 0.6f },  // 1: right whisker
            { Vector2Rotate(dir, -whiskerAngle),  lookAhead * 0.6f },  // 2: left whisker
        };

        float   closeness[3] = { 0.0f, 0.0f, 0.0f };   // 0 = no hit, 1 = touching
        Vector2 centerNormal = { 0.0f, 0.0f };

        for (int i = 0; i < 3; ++i) {
            // Nearest hit of THIS ray only
            RayHit best;
            auto consider = [&](const RayHit& h) {
                if (h.hit && (!best.hit || h.distance < best.distance)) best = h;
                };
            for (const Circle& c : world.obstacles)
                consider(RayVsCircle(pos, defs[i].dir, defs[i].length, c, agent.radius));
            for (const Wall& w : world.walls)
                consider(RayVsWall(pos, defs[i].dir, defs[i].length, w));

            whiskers[i] = { pos, Vector2Add(pos, Vector2Scale(defs[i].dir, defs[i].length)), best.hit };
            if (!best.hit) continue;

            closeness[i] = 1.0f - best.distance / defs[i].length;
            if (i == 0) centerNormal = best.normal;
        }

        if (closeness[0] <= 0.0f && closeness[1] <= 0.0f && closeness[2] <= 0.0f)
            return {};                                   // path is clear: say nothing

        // Positive = turn right, negative = turn left
        float side = closeness[2] - closeness[1];        // left hit -> right, right hit -> left
        if (closeness[0] > 0.0f) {
            float facing = Vector2DotProduct(centerNormal, right);
            float sign = (fabsf(facing) < 0.01f || facing > 0.0f) ? 1.0f : -1.0f;
            side += sign * closeness[0];                 // surface faces right -> turn right
        }
        float turn = (side >= 0.0f) ? 1.0f : -1.0f;      // only the sign matters

        steerDir = Vector2Scale(right, turn);
        steering = true;

        SteeringOutput out;
        out.linearAcceleration = Vector2Scale(steerDir, agent.maxAcceleration);
        return out;
    }

    void DebugDraw() const override
    {
        for (const Whisker& w : whiskers)
            DrawLineV(w.start, w.end, w.hit ? RED : Fade(RAYWHITE, 0.5f));
        if (steering)
            DrawLineEx(whiskers[0].start,
                Vector2Add(whiskers[0].start, Vector2Scale(steerDir, 50.0f)),
                3.0f, ORANGE);
    }

    float lookAhead;      // length of the center ray (px)
    float whiskerAngle;   // angle of the side rays (radians)

private:
    struct Whisker { Vector2 start{}, end{}; bool hit = false; };
    Whisker whiskers[3];
    Vector2 steerDir = { 0, 0 };
    bool    steering = false;
};



class PathFollowBehavior : public SteeringBehavior {
public:
    PathFollowBehavior(const Path* path, float pathOffset = 50.0f) : path(path), pathOffset(pathOffset) { }

    SteeringOutput GetSteering(const Agent& agent, const World&) override
    {
        // 1. Where on the path am I closest to? (a distance in px along the path)
        float param = path->GetParam(agent.kinematic.position);
        closest = path->GetPosition(param);

        // 2. The carrot: a point pathOffset px further along the path
        target = path->GetPosition(param + pathOffset);

        // 3. Seek the carrot
        return Steering::Seek(agent, target);
    }

    void DebugDraw() const override
    {
        DrawCircleV(closest, 5.0f, RED);       // closest point on the path
        DrawCircleV(target, 6.0f, ORANGE);     // the carrot being chased
    }

    const Path* path;       // shared, read-only
    float pathOffset;       // carrot distance ahead (px)

private:
    Vector2 closest = { 0, 0 };
    Vector2 target = { 0, 0 };
};


class FormationSlotBehavior : public SteeringBehavior {
public:
    FormationSlotBehavior(const Formation* formation, int slot, float slowRadius = 80.0f, float timeToTarget = 0.1f) : formation(formation), slot(slot), slowRadius(slowRadius), timeToTarget(timeToTarget) {}

    SteeringOutput GetSteering(const Agent& agent, const World&) override
    {
        slotPos = formation->SlotWorldPosition(slot);
        Vector2 toSlot = Vector2Subtract(slotPos, agent.kinematic.position);
        float   dist = Vector2Length(toSlot);

        // Start from the slot's own velocity (the leader's)...
        Vector2 desired = formation->leader->kinematic.velocity;

        // ...and add a closing velocity toward the slot that fades to 0 on arrival
        if (dist > 0.0001f) {
            float closing = fminf(dist / slowRadius, 1.0f) * agent.maxSpeed;
            desired = Vector2Add(desired, Vector2Scale(Vector2Normalize(toSlot), closing));
        }

        // Same as Arrive: acceleration needed to reach the desired velocity
        Vector2 accel = Vector2Scale(
            Vector2Subtract(desired, agent.kinematic.velocity), 1.0f / timeToTarget);
        if (Vector2Length(accel) > agent.maxAcceleration)
            accel = Vector2Scale(Vector2Normalize(accel), agent.maxAcceleration);

        SteeringOutput out;
        out.linearAcceleration = accel;
        return out;
    }

    const Formation* formation;   // not owned
    int   slot;
    float slowRadius;
    float timeToTarget;

private:
    Vector2 slotPos = { 0, 0 };
};