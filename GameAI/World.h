#pragma once
#include <vector>
#include "raylib.h"

class Agent;


// Placeholders for the avoidance step; unused for now
struct Circle { Vector2 center; float radius; };
struct Wall { Vector2 a; Vector2 b; };

struct World {
    float deltaTime = 0.0f;          // set by main every frame
    std::vector<Agent*> agents;      
    std::vector<Circle> obstacles;
    std::vector<Wall>   walls;
};