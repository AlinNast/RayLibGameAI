#include "raylib.h"
#include "raymath.h"
#include <vector>
#include "Agent.h"
#include "World.h"
#include "Behaviors.h"
#include "CompositeSteeringBehaviors.h"

enum class Mode { Seek, Flee, Arrive, Pursue, Evade, Wander, Stop };
const float slowRadius = 200.0f;
const float targetRadius = 10.0f;
Vector2 lastMouse = { 0.0f, 0.0f };


int main()
{
    InitWindow(1200, 900, "Steering Behaviors");
    SetTargetFPS(60);

	// --- Setup ---
	World world;
    ///// --- Mouse as a target with kinematic data ---
    Kinematic mouseTarget;
    mouseTarget.position = GetMousePosition();
    Vector2 lastMouse = mouseTarget.position;

    ///// --- Agents ---
    Agent agent({ 400.0f, 300.0f });
    world.agents.push_back(&agent);

    ///// --- Behaviors (created here, they must outlive the agents using them) ---
    SeekBehavior   seek(&mouseTarget);
    FleeBehavior   flee(&mouseTarget);
    ArriveBehavior arrive(&mouseTarget, 10.0f, 200.0f);
    PursueBehavior pursue(&mouseTarget);
    EvadeBehavior  evade(&mouseTarget);
    WanderBehavior wander;
    BrakeBehavior  brake;

    agent.behavior = &brake;
    const char* label = "0 - Brake";

    while (!WindowShouldClose()) {
		float delta = GetFrameTime(); // seconds since last frame
		world.deltaTime = delta;

        // --- Keyboad Input ---
        if (IsKeyPressed(KEY_ONE)) { agent.behavior = &seek;   label = "1 - Seek"; }
		if (IsKeyPressed(KEY_TWO)) { agent.behavior = &flee;   label = "2 - Flee"; }
		if (IsKeyPressed(KEY_THREE)) { agent.behavior = &arrive; label = "3 - Arrive"; }
		if (IsKeyPressed(KEY_FOUR)) { agent.behavior = &pursue; label = "4 - Pursue"; }
		if (IsKeyPressed(KEY_FIVE)) { agent.behavior = &evade;  label = "5 - Evade"; }
		if (IsKeyPressed(KEY_SIX)) { agent.behavior = &wander; label = "6 - Wander"; }
		if (IsKeyPressed(KEY_ZERO)) { agent.behavior = &brake;  label = "0 - Brake"; }


		// --- Update mouse target ---
        Vector2 mouse = GetMousePosition();
        Vector2 rawVel = (delta > 0.0f) ? Vector2Scale(Vector2Subtract(mouse, lastMouse), 1.0f / delta) : Vector2Zero();
        mouseTarget.velocity = Vector2Lerp(mouseTarget.velocity, rawVel, 0.2f);  // smooth the velocity of the mouse target from the actual mouse velocity
        mouseTarget.position = mouse;
        lastMouse = mouse;

        

        // --- Agent: apply steering ---
		//// --- Compute steering for all agents in the world ---
        std::vector<SteeringOutput> outputs; // holds results
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
        DrawText(label, 10, 10, 20, RAYWHITE);

        //// --- Objects ---
        for (Agent* a : world.agents) a->Draw();


		//// --- Debug ---
        DrawCircleV(mouse, 8.0f, Fade(SKYBLUE, 0.7f));
        DrawCircleLinesV(mouse, 8.0f, RAYWHITE);
        if (agent.behavior == &arrive) {
            DrawCircleLinesV(mouse, arrive.slowRadius, Fade(YELLOW, 0.6f));
            DrawCircleLinesV(mouse, arrive.targetRadius, Fade(RED, 0.8f));
        }
        if (agent.behavior == &wander) {
            DrawCircleLinesV(wander.data.circleCenter, wander.data.radius, Fade(YELLOW, 0.6f));
            DrawCircleV(wander.data.target, 5.0f, RED);
            DrawLineV(agent.kinematic.position, wander.data.target, Fade(RAYWHITE, 0.4f));
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
