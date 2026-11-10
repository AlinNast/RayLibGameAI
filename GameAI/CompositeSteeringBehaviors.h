#pragma once
#include <cmath>
#include <vector>
#include "raymath.h"
#include "SteeringBehavior.h"

// Weighted sum of several behaviors
class BlendedSteeringBehavior : public SteeringBehavior {
public:
    /// <summary>
    /// Add a behavior with a weight.
    /// </summary>
    /// <param name="behavior"></param>
    /// <param name="weight"></param>
    void AddBehavior(SteeringBehavior* behavior, float weight)
    {
        items.push_back({ behavior, weight });
    }

    /// <summary>
    /// Returns the weighted sum of all behaviors. (clamped) Comes from the interface
    /// </summary>
    /// <param name="agent"></param>
    /// <param name="world"></param>
    /// <returns></returns>
    SteeringOutput GetSteering(const Agent& agent, const World& world) override
    {
        SteeringOutput total;
        for (const Item& item : items) {
            SteeringOutput s = item.behavior->GetSteering(agent, world);
            total.linearAcceleration = Vector2Add(total.linearAcceleration,
                Vector2Scale(s.linearAcceleration, item.weight));
            total.angularAcceleration += s.angularAcceleration * item.weight;
        }

        // The sum can exceed what the agent can do, so clamp it
        if (Vector2Length(total.linearAcceleration) > agent.maxAcceleration)
            total.linearAcceleration = Vector2Scale(
                Vector2Normalize(total.linearAcceleration), agent.maxAcceleration);

        return total;
    }

private:
    struct Item { SteeringBehavior* behavior; float weight; };
	std::vector<Item> items; // holds the behaviors and their weights
};

// First behavior that has something to say wins
class PrioritySteeringBehavior : public SteeringBehavior {
public:
    /// <summary>
    /// Add a behavior to the list of behaviors.
    /// </summary>
    /// <param name="behavior"></param>
    void Add(SteeringBehavior* behavior) { items.push_back(behavior); }

    /// <summary>
    /// Returns the first behavior that has a non-zero steering output. (clamped) 
    /// </summary>
    /// <param name="agent"></param>
    /// <param name="world"></param>
    /// <returns></returns>
    SteeringOutput GetSteering(const Agent& agent, const World& world) override
    {
        SteeringOutput result;
        for (SteeringBehavior* item : items) {
            result = item->GetSteering(agent, world);
            if (Vector2Length(result.linearAcceleration) > threshold ||
                fabsf(result.angularAcceleration) > threshold)
                return result;
        }
        return result;   // nobody spoke up: return the last (usually "do nothing")
    }

    float threshold = 0.001f;

private:
    std::vector<SteeringBehavior*> items;
};