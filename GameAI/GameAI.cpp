#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <memory>
#include "Agent.h"
#include "World.h"
#include "Behaviors.h"
#include "CompositeSteeringBehaviors.h"

enum class BehaviorType { Seek, Flee, Arrive, Pursue, Evade, Wander, Brake, WanderApart, WanderAvoid };


static void SetBehavior(Agent& agent, BehaviorType type, const Kinematic* target)
{
    switch (type) {
    case BehaviorType::Seek:
        agent.behavior = std::make_unique<SeekBehavior>(target);
        agent.behaviorLabel = "1 - Seek"; break;
    case BehaviorType::Flee:
        agent.behavior = std::make_unique<FleeBehavior>(target);
        agent.behaviorLabel = "2 - Flee"; break;
    case BehaviorType::Arrive:
        agent.behavior = std::make_unique<ArriveBehavior>(target, 10.0f, 200.0f);
        agent.behaviorLabel = "3 - Arrive"; break;
    case BehaviorType::Pursue:
        agent.behavior = std::make_unique<PursueBehavior>(target);
        agent.behaviorLabel = "4 - Pursue"; break;
    case BehaviorType::Evade:
        agent.behavior = std::make_unique<EvadeBehavior>(target);
        agent.behaviorLabel = "5 - Evade"; break;
    case BehaviorType::Wander:
        agent.behavior = std::make_unique<WanderBehavior>();
        agent.behaviorLabel = "6 - Wander"; break;
    case BehaviorType::WanderApart: {
        auto blend = std::make_unique<BlendedSteeringBehavior>();
        blend->AddBehavior(std::make_unique<WanderBehavior>(), 1.0f);
        blend->AddBehavior(std::make_unique<SeparationBehavior>(100.0f, 200000.0f), 10.0f);
        agent.behavior = std::move(blend);
        agent.behaviorLabel = "7 - Wander + Separation"; break;
    }
    case BehaviorType::WanderAvoid: {
        auto blend = std::make_unique<PrioritySteeringBehavior>();
        blend->Add(std::make_unique<CollisionAvoidanceBehavior>(10.0f, 15.0f));
        blend->Add(std::make_unique<WanderBehavior>());
        agent.behavior = std::move(blend);
        agent.behaviorLabel = "8 - Wander + Collision Avoidance"; break;
    }
    case BehaviorType::Brake:
        agent.behavior = std::make_unique<BrakeBehavior>();
        agent.behaviorLabel = "0 - Brake"; break;
    }
}


static Agent* PickAgent(const std::vector<std::unique_ptr<Agent>>& agents,
    Vector2 point, float pickRadius)
{
    for (const auto& a : agents)
        if (CheckCollisionPointCircle(point, a->kinematic.position, pickRadius))
            return a.get();
    return nullptr;
}



int main()
{
    InitWindow(1200, 900, "Steering Behaviors");
    SetTargetFPS(60);

	// --- Setup ---
	World world;
    std::vector<std::unique_ptr<Agent>> agents;   // the owner
    Agent* selected = nullptr;

    ///// --- Mouse as a target with kinematic data ---
    Kinematic mouseTarget;
    mouseTarget.position = GetMousePosition();
    Vector2 lastMouse = mouseTarget.position;

    ///// --- Agents ---
    auto spawn = [&](Vector2 pos) {
        agents.push_back(std::make_unique<Agent>(pos));
        Agent* a = agents.back().get();
        world.agents.push_back(a);
        SetBehavior(*a, BehaviorType::Brake, &mouseTarget);
        selected = a;                              // new agent becomes the active one
    };
    spawn({ 400.0f, 300.0f });

    

    while (!WindowShouldClose()) {
		float delta = GetFrameTime(); // seconds since last frame
		world.deltaTime = delta;

		// --- Update mouse target ---
        Vector2 mouse = GetMousePosition();
        Vector2 rawVel = (delta > 0.0f) ? Vector2Scale(Vector2Subtract(mouse, lastMouse), 1.0f / delta) : Vector2Zero();
        mouseTarget.velocity = Vector2Lerp(mouseTarget.velocity, rawVel, 0.2f);  // smooth the velocity of the mouse target from the actual mouse velocity
        mouseTarget.position = mouse;
        lastMouse = mouse;

		// --- Spawn input ---
        if (IsKeyPressed(KEY_N)) spawn(mouse);

		// --- Select input ---
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
            selected = PickAgent(agents, mouse, 25.0f);   // nullptr = deselect

        // --- Command Input ---
        if (selected) {
            if (IsKeyPressed(KEY_ONE))   SetBehavior(*selected, BehaviorType::Seek, &mouseTarget);
            if (IsKeyPressed(KEY_TWO))   SetBehavior(*selected, BehaviorType::Flee, &mouseTarget);
            if (IsKeyPressed(KEY_THREE)) SetBehavior(*selected, BehaviorType::Arrive, &mouseTarget);
            if (IsKeyPressed(KEY_FOUR))  SetBehavior(*selected, BehaviorType::Pursue, &mouseTarget);
            if (IsKeyPressed(KEY_FIVE))  SetBehavior(*selected, BehaviorType::Evade, &mouseTarget);
            if (IsKeyPressed(KEY_SIX))   SetBehavior(*selected, BehaviorType::Wander, &mouseTarget);
            if (IsKeyPressed(KEY_SEVEN)) SetBehavior(*selected, BehaviorType::WanderApart, &mouseTarget);
            if (IsKeyPressed(KEY_EIGHT)) SetBehavior(*selected, BehaviorType::WanderAvoid, &mouseTarget);
            if (IsKeyPressed(KEY_ZERO))  SetBehavior(*selected, BehaviorType::Brake, &mouseTarget);
        }

        // --- Delete input ---
        if (IsKeyPressed(KEY_D) && selected) {
            std::erase(world.agents, selected);                     // 1. remove it from the world
            std::erase_if(agents, [&](const std::unique_ptr<Agent>& a) {
                return a.get() == selected;                         // 2. destroy it (unique_ptr frees it)
                });
            selected = nullptr;                                     // 3. nothing dangling
        }



        

        // --- Agent: apply steering ---
        
        //// --- Compute steering for all agents in the world ---
        std::vector<SteeringOutput> outputs;
        outputs.reserve(world.agents.size());
        for (Agent* a : world.agents)
            outputs.push_back(a->ComputeSteering(world));

        //// --- Apply steering for all agents ---
        for (size_t i = 0; i < world.agents.size(); ++i)
            world.agents[i]->Update(outputs[i], delta);

        // --- Draw ---
        BeginDrawing();

        //// --- Environment --- 
        ClearBackground(DARKGREEN);
        DrawText(selected ? TextFormat("Selected: %s", selected->behaviorLabel) : "Nothing selected (click an agent, N = spawn)", 10, 10, 20, RAYWHITE);
        DrawText(TextFormat("Agents: %d", (int)agents.size()), 10, 35, 20, RAYWHITE);

        //// --- Objects ---
        for (const auto& a : agents) a->Draw(a.get() == selected);


		//// --- Debug ---
        DrawCircleV(mouse, 8.0f, Fade(SKYBLUE, 0.7f));
        DrawCircleLinesV(mouse, 8.0f, RAYWHITE);
        if (selected) {
            if (auto* ar = dynamic_cast<ArriveBehavior*>(selected->behavior.get())) {
                DrawCircleLinesV(mouse, ar->slowRadius, Fade(YELLOW, 0.6f));
                DrawCircleLinesV(mouse, ar->targetRadius, Fade(RED, 0.8f));
            }
            if (auto* wa = dynamic_cast<WanderBehavior*>(selected->behavior.get())) {
                DrawCircleLinesV(wa->data.circleCenter, wa->data.radius, Fade(YELLOW, 0.6f));
                DrawCircleV(wa->data.target, 5.0f, RED);
                DrawLineV(selected->kinematic.position, wa->data.target, Fade(RAYWHITE, 0.4f));
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
