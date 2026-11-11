#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <memory>
#include "Agent.h"
#include "World.h"
#include "Behaviors.h"
#include "CompositeSteeringBehaviors.h"
#include "Formation.h"

enum class BehaviorType { Seek, Flee, Arrive, Pursue, Evade, Wander, Brake, WanderApart, WanderAvoid, PathFollow };

static void DrawControls()
{
    const char* lines[] = {
        "Click: select   N: spawn at cursor   D: delete selected",
        "1 Seek  2 Flee  3 Arrive  4 Pursue  5 Evade  6 Wander  7 Wander+Separation  8 Wander+Avoid  9 Path  0 Brake"   // for the next step
    };
    const int count = sizeof(lines) / sizeof(lines[0]);
    const int fontSize = 18;
    const int lineH = 24;
    const int barH = count * lineH + 12;
    const int top = GetScreenHeight() - barH;

    DrawRectangle(0, top, GetScreenWidth(), barH, Fade(BLACK, 0.55f));
    for (int i = 0; i < count; ++i)
        DrawText(lines[i], 10, top + 8 + i * lineH, fontSize, RAYWHITE);
}

static void SetBehavior(Agent& agent, BehaviorType type, const Kinematic* target, const Path& path)
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

        auto priority = std::make_unique<PrioritySteeringBehavior>();
        priority->Add(std::make_unique<ObstacleAvoidanceBehavior>());   // 1st: never hit a wall
        priority->Add(std::move(blend));                                // 2nd: normal behavior
        agent.behavior = std::move(priority);
        agent.behaviorLabel = "7 - Wander + Separation"; break;
    }
    case BehaviorType::WanderAvoid: {
        auto priority = std::make_unique<PrioritySteeringBehavior>();
		priority->Add(std::make_unique<ObstacleAvoidanceBehavior>());   // 1st: never hit a wall
		priority->Add(std::make_unique<CollisionAvoidanceBehavior>(10.0f, 15.0f)); // 2nd: never hit another agent
		priority->Add(std::make_unique<WanderBehavior>());              /// 3rd: normal behavior
        agent.behavior = std::move(priority);
        agent.behaviorLabel = "8 - Wander + Collision Avoidance"; break;
    }
    case BehaviorType::PathFollow: {
        auto travel = std::make_unique<BlendedSteeringBehavior>();
        travel->AddBehavior(std::make_unique<PathFollowBehavior>(&path, 50.0f), 1.0f);
        travel->AddBehavior(std::make_unique<SeparationBehavior>(100.0f, 200000.0f), 3.0f);

        auto brain = std::make_unique<PrioritySteeringBehavior>();
        brain->Add(std::make_unique<ObstacleAvoidanceBehavior>());                 // 1st: geometry
        brain->Add(std::make_unique<CollisionAvoidanceBehavior>(2.0f, 10.0f));     // 2nd: other agents
        brain->Add(std::move(travel));                                             // 3rd: follow the path
        agent.behavior = std::move(brain);
        agent.behaviorLabel = "9 - Path Follow"; break;
    }
    case BehaviorType::Brake:
        agent.behavior = std::make_unique<BrakeBehavior>();
        agent.behaviorLabel = "0 - Brake"; break;
    }
}


static void StartFormation(std::unique_ptr<Formation>& formation, Agent& leader,
    const Kinematic* target)
{
    formation = std::make_unique<Formation>();
    formation->leader = &leader;
    formation->savedLeaderMaxSpeed = leader.maxSpeed;
    leader.maxSpeed = 140.0f;                      // slower than followers (see "why")

    auto brain = std::make_unique<PrioritySteeringBehavior>();
    brain->Add(std::make_unique<ObstacleAvoidanceBehavior>());
    brain->Add(std::make_unique<ArriveBehavior>(target, 10.0f, 200.0f));
    leader.behavior = std::move(brain);
    leader.behaviorLabel = "F - Formation leader (Arrive)";
}

static bool JoinFormation(Formation& formation, Agent& agent)
{
    int slot = formation.FirstFreeSlot();
    if (slot < 0 || formation.Contains(&agent)) return false;

    formation.slots[slot] = &agent;
    auto brain = std::make_unique<PrioritySteeringBehavior>();
    brain->Add(std::make_unique<ObstacleAvoidanceBehavior>());
    brain->Add(std::make_unique<FormationSlotBehavior>(&formation, slot));
    agent.behavior = std::move(brain);
    agent.behaviorLabel = "F - Formation member";
    return true;
}

// Everyone gets the same new behavior, then the formation is destroyed.
static void DissolveFormation(std::unique_ptr<Formation>& formation, BehaviorType type,
    const Kinematic* target, const Path& path)
{
    if (!formation) return;
    formation->leader->maxSpeed = formation->savedLeaderMaxSpeed;
    SetBehavior(*formation->leader, type, target, path);
    for (Agent* f : formation->slots)
        if (f) SetBehavior(*f, type, target, path);
    formation.reset();   // safe now: no behavior points at it any more
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
    
    world.obstacles.push_back({ { 450.0f, 450.0f }, 70.0f });
    world.obstacles.push_back({ { 850.0f, 250.0f }, 50.0f });
    world.walls.push_back({ { 700.0f, 500.0f }, { 700.0f, 800.0f } });
    
    Path path({
    { 150.0f, 150.0f }, { 600.0f, 100.0f }, { 1050.0f, 150.0f }, { 1100.0f, 500.0f },
    { 1000.0f, 820.0f }, { 550.0f, 840.0f }, { 200.0f, 780.0f }, { 300.0f, 450.0f }
    }, true);

    std::unique_ptr<Formation> formation;

    ///// --- Mouse as a target with kinematic data ---
    Kinematic mouseTarget;
    mouseTarget.position = GetMousePosition();
    Vector2 lastMouse = mouseTarget.position;

    ///// --- Agents ---
    auto spawn = [&](Vector2 pos) {
        agents.push_back(std::make_unique<Agent>(pos));
        Agent* a = agents.back().get();
        world.agents.push_back(a);
        SetBehavior(*a, BehaviorType::Brake, &mouseTarget, path);
        if (!formation) selected = a;                              // new agent becomes the active one
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
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Agent* picked = PickAgent(agents, mouse, 25.0f);
            if (formation) {
                if (picked) JoinFormation(*formation, *picked);   // ignored if full or already in
            }
            else {
                selected = picked;
            }
        }

        // --- Delete input ---
        if (IsKeyPressed(KEY_D) && selected) {
            DissolveFormation(formation, BehaviorType::Brake, &mouseTarget, path);  // first!
            std::erase(world.agents, selected);
            std::erase_if(agents, [&](const std::unique_ptr<Agent>& a) { return a.get() == selected; });
            selected = nullptr;
        }

        // --- Command Input ---
        if (selected) {
            // With a formation active, a behavior key disbands it and applies to everyone
            auto command = [&](BehaviorType t) {
                if (formation) DissolveFormation(formation, t, &mouseTarget, path);
                else           SetBehavior(*selected, t, &mouseTarget, path);
                };

            if (IsKeyPressed(KEY_F) && !formation) StartFormation(formation, *selected, &mouseTarget);

            if (IsKeyPressed(KEY_ONE))   command(BehaviorType::Seek);
            if (IsKeyPressed(KEY_TWO))   command(BehaviorType::Flee);
            if (IsKeyPressed(KEY_THREE)) command(BehaviorType::Arrive);
            if (IsKeyPressed(KEY_FOUR))  command(BehaviorType::Pursue);
            if (IsKeyPressed(KEY_FIVE))  command(BehaviorType::Evade);
            if (IsKeyPressed(KEY_SIX))   command(BehaviorType::Wander);
            if (IsKeyPressed(KEY_SEVEN)) command(BehaviorType::WanderApart);
            if (IsKeyPressed(KEY_EIGHT)) command(BehaviorType::WanderAvoid);
            if (IsKeyPressed(KEY_NINE))  command(BehaviorType::PathFollow);
            if (IsKeyPressed(KEY_ZERO))  command(BehaviorType::Brake);

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
        for (const Circle& c : world.obstacles) {
            DrawCircleV(c.center, c.radius, DARKGRAY);
            DrawCircleLinesV(c.center, c.radius, RAYWHITE);
        }
        for (const Wall& w : world.walls)
            DrawLineEx(w.a, w.b, 6.0f, LIGHTGRAY);
        path.Draw();


		//// --- Debug ---
        DrawCircleV(mouse, 8.0f, Fade(SKYBLUE, 0.7f));
        DrawCircleLinesV(mouse, 8.0f, RAYWHITE);
        DrawControls();
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
        if (selected && selected->behavior) selected->behavior->DebugDraw();

        if (formation) {
            for (int i = 0; i < Formation::MaxFollowers; ++i) {
                Vector2 p = formation->SlotWorldPosition(i);
                DrawCircleLinesV(p, 14.0f, Fade(YELLOW, 0.7f));
                if (formation->slots[i])
                    DrawLineV(p, formation->slots[i]->kinematic.position, Fade(RAYWHITE, 0.4f));
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}


