#include "raylib.h"
#include "raymath.h"
#include "Agent.h"
#include "Steering.h"
#include <string>

enum class Mode { Seek, Flee, Arrive, Pursue, Stop };
const float slowRadius = 200.0f;
const float targetRadius = 10.0f;
Vector2 lastMouse = { 0.0f, 0.0f };


int main()
{
    InitWindow(1200, 900, "Steering Behaviors");
    SetTargetFPS(60);

    Agent agent({ 400.0f, 300.0f });
    Mode mode = Mode::Stop;

    while (!WindowShouldClose()) {
		float delta = GetFrameTime(); // seconds since last frame

        // --- Input ---
        if (IsKeyPressed(KEY_ONE)) mode = Mode::Seek;
		if (IsKeyPressed(KEY_TWO)) mode = Mode::Flee;
		if (IsKeyPressed(KEY_THREE)) mode = Mode::Arrive;
		if (IsKeyPressed(KEY_FOUR)) mode = Mode::Pursue;
        if (IsKeyPressed(KEY_ZERO)) mode = Mode::Stop;

        Vector2 mouse = GetMousePosition();
        
		Vector2 mouseVel = Vector2Scale(Vector2Subtract(mouse, lastMouse), 1.0f / delta); // calculate mouse velocity based on change in position over time
        lastMouse = mouse; // needed for pursue and evade ( later if the target is not the mouse it can be changed with the vel of the target)


        // --- Controller ---
        const char* label = "";

		SteeringOutput steering; // will hold the steering output for the current mode
        switch (mode) {
        case Mode::Seek:    steering = Steering::Seek(agent, mouse); label = "1 - Seek";    break;
		case Mode::Flee:    steering = Steering::Flee(agent, mouse); label = "2 - Flee";    break;
		case Mode::Arrive:  steering = Steering::Arrive(agent, mouse, targetRadius, slowRadius); label = "3 - Arrive";  break;
		case Mode::Pursue:  steering = Steering::Pursue(agent, mouse, mouseVel); label = "4 - Pursue";  break;
        case Mode::Stop:    steering = Steering::Brake(agent);       label = "0 - Brake";   break;
        }

        // --- Agent: apply steering ---
        agent.Update(steering, delta);

        // --- Draw ---
        BeginDrawing();

        //// --- Environment --- 
        ClearBackground(DARKGREEN);
        DrawText(label, 10, 10, 20, RAYWHITE);

        //// --- Objects ---
        agent.Draw();


		//// --- Debug ---
        DrawCircleV(mouse, 8.0f, Fade(SKYBLUE, 0.7f));
        DrawCircleLinesV(mouse, 8.0f, RAYWHITE);
        if (mode == Mode::Arrive) {
            DrawCircleLinesV(mouse, slowRadius, Fade(YELLOW, 0.6f));
            DrawCircleLinesV(mouse, targetRadius, Fade(RED, 0.8f));
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
