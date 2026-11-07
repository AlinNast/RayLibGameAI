// GameAI.cpp : Defines the entry point for the application.
//
#include "raylib.h"
#include <cmath>
#include "GameAI.h"

int main()
{
    InitWindow(800, 600, "Steering Behaviors");
    SetTargetFPS(60);

    Vector2 pos = { 400.0f, 300.0f };
    float heading = 0.0f;  // radians, 0 = pointing right

    while (!WindowShouldClose()) {
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

        BeginDrawing();
        ClearBackground(RAYWHITE);
        DrawTriangle(tip, left, right, MAROON);  // counter-clockwise order
        DrawText("Hello raylib", 10, 10, 20, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
