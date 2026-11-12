#pragma once
#include "SteeringBehavior.h"
#include "Steering.h"
#include "raymath.h"
#include "Collision.h"
#include <cmath>
#include <functional>  ///std::less (safe pointer comparison)
#include <array>
#include "Path.h"
#include "Formation.h"


/// Behaviours is more like a wrapper for the Steering class, most of the math is done in the steering class, this wraps it up with the needed data.


/// <summary>
/// Seek behavior is simple, applies seek steering and returns the output
/// </summary>
class SeekBehavior : public SteeringBehavior {
public:
    explicit SeekBehavior(const Kinematic* target) : target(target) {}  // this is a constructor, after the dots is the member initializer list, i still dont understand what explicit means
    SteeringOutput GetSteering(const Agent& agent, const World&) override   
    {
        return Steering::Seek(agent, target->position);
    }
    const Kinematic* target;
};


/// <summary>
/// Flee is the opposite of seek, it applies flee steering and returns the output
/// </summary>
class FleeBehavior : public SteeringBehavior {
public:
    explicit FleeBehavior(const Kinematic* target) : target(target) {}
    SteeringOutput GetSteering(const Agent& agent, const World&) override
    {
        return Steering::Flee(agent, target->position);
    }
    const Kinematic* target;
};


/// <summary>
/// Arrive uses radius to slow down or consider it arrived
/// </summary>
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
	float targetRadius; // the radius at which the agent will consider itself to have arrived at the target
	float slowRadius;  // the radius at which the agent will start to slow down as it approaches the target
    float timeToTarget = 0.1f;  // Smoothing factor for the arrival, smaller is sharper
};


/// <summary>
///  Pursue is like seek but it predicts where the target will be based on its velocity
/// </summary>
class PursueBehavior : public SteeringBehavior {
public:
    explicit PursueBehavior(const Kinematic* target) : target(target) {}
    SteeringOutput GetSteering(const Agent& agent, const World&) override
    {
        return Steering::Pursue(agent, target->position, target->velocity, maxPrediction);
    }
    const Kinematic* target;
	float maxPrediction = 1.0f; // how far ahead in time to predict the target's position, in seconds
};


/// <summary>
/// Like pursue but it flees
/// </summary>
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


/// <summary>
/// Contains the state for the wander behavior, one per agent and calls Wander steering with it
/// </summary>
class WanderBehavior : public SteeringBehavior {
public:
    SteeringOutput GetSteering(const Agent& agent, const World& world) override
    {
        return Steering::Wander(agent, data, world.deltaTime); 
    }
    Steering::WanderData data;   // the state now lives INSIDE the behavior - one per agent
};


/// <summary>
///  Slows down progressively and stays at rest
/// </summary>
class BrakeBehavior : public SteeringBehavior {
public:
    SteeringOutput GetSteering(const Agent& agent, const World&) override
    {
        return Steering::Brake(agent);
    }
};


/// <summary>
///  push away from every nehboring agent that is too close, Checks wi all agents this is O^2 algoritm 
/// looks at current positions (Reactive)
/// </summary>
class SeparationBehavior : public SteeringBehavior {
public:
	SeparationBehavior(float thresholdRadius, float pushStrenght) : thresholdRadius(thresholdRadius), pushStrenght(pushStrenght) {} // constructor

    SteeringOutput GetSteering(const Agent& agent, const World& world) override // check with all the agents in the world
    {
        SteeringOutput out;

        for (const Agent* other : world.agents) {
            // Edge cases
            if (other == &agent) continue;                  // don't flee from yourself

            Vector2 dirToPush = Vector2Subtract(agent.kinematic.position, other->kinematic.position);
			float dist = Vector2Length(dirToPush);               // distance to the other agent

            if (dist >= thresholdRadius) continue;                // outside personal space

            if (dist < 0.0001f) {
                // Exactly* on top of each other: there's no direction to flee in.
                // Both agents run this code, so they must pick OPPOSITE directions
                // or they would move together forever.
                dirToPush = std::less<const Agent*>{}(&agent, other) ? Vector2{ 1, 0 } : Vector2{ -1, 0 };
                dist = 0.0001f;
            }

            
            // The actual push
            float strength = fminf(pushStrenght / (dist * dist), agent.maxAcceleration); // push strenght/dist^2, but clamped if its too big
            out.linearAcceleration = Vector2Add(out.linearAcceleration,Vector2Scale(Vector2Normalize(dirToPush), strength));   // ||dir to push|| * strength + current lAcc 
        }

        // lAcc adds all the vectors from the other agents, so if it gets to big clamp it
        if (Vector2Length(out.linearAcceleration) > agent.maxAcceleration)
            out.linearAcceleration = Vector2Scale(Vector2Normalize(out.linearAcceleration), agent.maxAcceleration);

        return out;
    }

    float thresholdRadius;   // personal-space radius (px)
    float pushStrenght;       // push strength coefficient
};


/// <summary>
///  avoids collision with other agents future positions, looks at current positions and velocities (Predictive)
/// </summary>
class CollisionAvoidanceBehavior : public SteeringBehavior {
public:
    explicit CollisionAvoidanceBehavior(float maxLookAhead = 2.0f, float margin = 10.0f) : maxLookAhead(maxLookAhead), margin(margin) { }

    SteeringOutput GetSteering(const Agent& agent, const World& world) override
    {
        bool    found = false;
        float   shortestTime = maxLookAhead;   // ignore collisions further away than this
		Vector2 threatRelPos = { 0, 0 };  // will store the closest threat's relative position and velocity
        Vector2 threatRelVel = { 0, 0 };

        for (const Agent* other : world.agents) {
			// Edge cases
            if (other == &agent) continue;

            Vector2 relPos = Vector2Subtract(other->kinematic.position, agent.kinematic.position);
            Vector2 relVel = Vector2Subtract(other->kinematic.velocity, agent.kinematic.velocity);
			float combined = agent.radius + other->radius + margin; // the distance at which we consider a collision to be happening
            float dist = Vector2Length(relPos);

            float t = 0.0f;                          // 0 means "already overlapping"
			if (dist >= combined) {                 // not overlaping yet go to prediction
                float relSpeedSq = Vector2DotProduct(relVel, relVel);
                if (relSpeedSq < 0.0001f) continue;  // same velocity: the gap never changes

				t = -Vector2DotProduct(relPos, relVel) / relSpeedSq; // time to closest approach
                if (t <= 0.0f) continue;             // moving apart

                Vector2 closest = Vector2Add(relPos, Vector2Scale(relVel, t));  
                if (Vector2Length(closest) >= combined) continue;   // will pass safely
            }

            if (t >= shortestTime) continue;         // not the most urgent one
			found = true;   // flag that there is a threat, and store the data for the closest one
			shortestTime = t;  // the time to closest approach of the most urgent threat
			threatRelPos = relPos;   // the relative position of the most urgent threat
            threatRelVel = relVel;
        }

        if (!found) return {};                       // nothing to dodge: say nothing

        // Where the threat will be relative to this at the moment of closest approach
        Vector2 avoidFrom = Vector2Add(threatRelPos, Vector2Scale(threatRelVel, shortestTime));

        if (shortestTime == 0.0f)
            avoidFrom = threatRelPos;                // already touching: flee from where it is now
        else if (Vector2Length(avoidFrom) < 0.0001f)
            avoidFrom = { -threatRelVel.y, threatRelVel.x };   // dead-centre head-on: sidestep

        SteeringOutput out;
		out.linearAcceleration = Vector2Scale(Vector2Normalize(avoidFrom), -agent.maxAcceleration);  // max acceleration away from the threat
        return out;
    }

    float maxLookAhead;   // seconds to look into the future
    float margin;         // extra personal space added to both radii (px)
};




class ObstacleAvoidanceBehavior : public SteeringBehavior {
public:
    ObstacleAvoidanceBehavior(float lookAhead = 120.0f, float whiskerAngleDeg = 35.0f) : lookAhead(lookAhead), whiskerAngle(whiskerAngleDeg* DEG2RAD) { } // constructor and also chanhe the angle from degrees to radians

    SteeringOutput GetSteering(const Agent& agent, const World& world) override
    {
        debug = false; // debug flag
        const Vector2 pos = agent.kinematic.position;

        // Step 1, get the vectors
        // Heading: along the velocity, or the facing if almost stationary
        Vector2 dir = (Vector2Length(agent.kinematic.velocity) > 1.0f) ? Vector2Normalize(agent.kinematic.velocity) : agent.Forward();
        const Vector2 right = Vector2Rotate(dir, PI / 2.0f);   // the agent's right-hand side

		// Step 2, cast the rays
        struct RayDef { Vector2 dir; float length; };
        const RayDef defs[3] = {
            { dir,                                lookAhead },         // 0: center
            { Vector2Rotate(dir,  whiskerAngle),  lookAhead * 0.6f },  // 1: right whisker
            { Vector2Rotate(dir, -whiskerAngle),  lookAhead * 0.6f },  // 2: left whisker
        };

        float   closeness[3] = { 0.0f, 0.0f, 0.0f };   // 0 = no hit, 1 = touching
		Vector2 centerNormal = { 0.0f, 0.0f }; // the normal of the center hit, if any

        // Step 3 : go over the rays
        for (int i = 0; i < 3; ++i) {
            // Nearest hit of THIS ray only
            RayHit best;
			auto consider = [&](const RayHit& h) { // lambda function to consider a hit and update the best hit if it's closer
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
        debug = true;

        SteeringOutput out;
        out.linearAcceleration = Vector2Scale(steerDir, agent.maxAcceleration);
        return out;
    }

    void DebugDraw() const override
    {
        for (const Whisker& w : whiskers)
            DrawLineV(w.start, w.end, w.hit ? RED : Fade(RAYWHITE, 0.5f));
        if (debug)
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
    bool    debug = true;
};



class PathFollowBehavior : public SteeringBehavior {
public:
    PathFollowBehavior(const Path* path, float pathOffset = 50.0f) : path(path), pathOffset(pathOffset) { }

    SteeringOutput GetSteering(const Agent& agent, const World&) override
    {
        // 1. Where on the path am I closest to? (a distance in px along the path)
		float param = path->GetParam(agent.kinematic.position); // distance along the path of the closest point
		closest = path->GetPosition(param); // closest point on the path

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
		slotPos = formation->SlotWorldPosition(slot); // get the world position of the slot in the formation based on the index
        Vector2 toSlot = Vector2Subtract(slotPos, agent.kinematic.position);
        float   dist = Vector2Length(toSlot);

        // Start from the slot's own velocity (the leader's)...
        Vector2 desired = formation->leader->kinematic.velocity;

        // add a closing velocity toward the slot that fades to 0 on arrival
        if (dist > 0.0001f) {
            float closing = fminf(dist / slowRadius, 1.0f) * agent.maxSpeed;
            desired = Vector2Add(desired, Vector2Scale(Vector2Normalize(toSlot), closing));
        }

        // Same as Arrive: acceleration needed to reach the desired velocity
        Vector2 accel = Vector2Scale(Vector2Subtract(desired, agent.kinematic.velocity), 1.0f / timeToTarget);
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

