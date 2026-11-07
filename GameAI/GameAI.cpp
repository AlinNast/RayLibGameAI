// GameAI.cpp : Defines the entry point for the application.
//
#include "raylib.h"
#include <cmath>
#include <string>
#include "GameAI.h"

int main()
{
    InitWindow(1200, 900, "Steering Behaviors");
    SetTargetFPS(60);

    Vector2 pos = { 400.0f, 300.0f };
    float heading = 0.0f;  // radians, 0 = pointing right

    std::string label = "Press 1 or 2";

    while (!WindowShouldClose()) {

        // --- Input ---
        if (IsKeyPressed(KEY_ONE))  label = "1";
        if (IsKeyPressed(KEY_TWO))  label = "2";

        Vector2 mouse = GetMousePosition();


        // Slowly spin so you can see it's a real triangle
        heading += 1.0f * GetFrameTime();

        const float size = 25.0f;
        const float back = 140.0f * DEG2RAD;

        Vector2 tip = { pos.x + cosf(heading) * size,
                         pos.y + sinf(heading) * size };
        Vector2 left = { pos.x + cosf(heading - back) * size,
                         pos.y + sinf(heading - back) * size };
        Vector2 right = { pos.x + cosf(heading + back) * size,
                         pos.y + sinf(heading + back) * size };


		// --- Draw ---
        BeginDrawing();
        ClearBackground(DARKGREEN);

        DrawTriangle(tip, left, right, MAROON);  // counter-clockwise order
        DrawText("Hello raylib", 10, 10, 20, DARKGRAY);

        DrawCircleV(mouse, 8.0f, Fade(SKYBLUE, 0.7f));
        DrawCircleLinesV(mouse, 8.0f, DARKBLUE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
