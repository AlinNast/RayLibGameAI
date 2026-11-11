#pragma once
#include "Agent.h"
#include "World.h"

/// <summary>
/// Interface for a steering behaviour to returns a steering output
/// </summary>
class SteeringBehavior {
public:
    virtual ~SteeringBehavior() = default;
    virtual SteeringOutput GetSteering(const Agent& agent, const World& world) = 0;
    virtual void DebugDraw() const {}      // optional: most behaviors draw nothing
};
