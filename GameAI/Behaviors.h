#pragma once
#include "SteeringBehavior.h"
#include "Steering.h"


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