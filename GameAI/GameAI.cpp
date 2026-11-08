#include "raylib.h"
#include "raymath.h"
#include "Agent.h"
#include "Steering.h"
#include <string>

enum class Mode { Seek, Flee, Stop };


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
        if (IsKeyPressed(KEY_ZERO)) mode = Mode::Stop;

        Vector2 mouse = GetMousePosition();


        // --- Controller: compute steering ---
        const char* label = "";
        SteeringOutput steering;
        switch (mode) {
        case Mode::Seek:    steering = Steering::Seek(agent, mouse); label = "1 - Seek";    break;
		case Mode::Flee:    steering = Steering::Flee(agent, mouse); label = "2 - Flee";    break;
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

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
