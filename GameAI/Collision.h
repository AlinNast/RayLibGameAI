#pragma once
#include <cmath>

#include "raymath.h"
#include "World.h"

struct RayHit {
    bool    hit = false;
    float   distance = 0.0f;       // from the ray origin
    Vector2 point = { 0, 0 };       // where the ray touches the surface
    Vector2 normal = { 0, 0 };   // unit vector pointing away from the surface
};

static float Cross2D(Vector2 a, Vector2 b) { return a.x * b.y - a.y * b.x; }


// Ray (origin + dir * t, 0 <= t <= length) against a circle.
// extraRadius inflates the circle by the agent's own size.
static RayHit RayVsCircle(Vector2 origin, Vector2 dir, float length,
    const Circle& c, float extraRadius)
{
    RayHit result;
    float r = c.radius + extraRadius;  // inflated radius

    //set up the quadratic  t² + 2bt + cc = 0
    Vector2 f = Vector2Subtract(origin, c.center);
    float b = Vector2DotProduct(f, dir);
    float cc = Vector2DotProduct(f, f) - r * r;

    if (cc < 0.0f) {                       // origin already inside the circle
        result.hit = true;
        result.distance = 0.0f;
        result.point = origin;
        result.normal = Vector2Normalize(f);
        return result;
    }

    float disc = b * b - cc;
    if (disc < 0.0f) return result;        // the ray's line misses the circle

    float t = -b - sqrtf(disc);            // nearer of the two intersections
    if (t < 0.0f || t > length) return result;

    result.hit = true;
    result.distance = t;
    result.point = Vector2Add(origin, Vector2Scale(dir, t));
    result.normal = Vector2Normalize(Vector2Subtract(result.point, c.center));
    return result;
}

// Ray against a wall segment a-b.
static RayHit RayVsWall(Vector2 origin, Vector2 dir, float length, const Wall& w)
{
    RayHit result;
    Vector2 e = Vector2Subtract(w.b, w.a);
    Vector2 ao = Vector2Subtract(w.a, origin);
    float   denom = Cross2D(dir, e);
    if (fabsf(denom) < 0.0001f) return result;     // parallel: never crosses

    float t = Cross2D(ao, e) / denom;            // distance along the ray
    float u = Cross2D(ao, dir) / denom;            // position along the wall, 0..1
    if (t < 0.0f || t > length || u < 0.0f || u > 1.0f) return result;

    Vector2 n = Vector2Normalize({ -e.y, e.x });   // a perpendicular of the wall
    if (Vector2DotProduct(n, dir) > 0.0f) n = Vector2Negate(n);   // face the agent's side

    result.hit = true;
    result.distance = t;
    result.point = Vector2Add(origin, Vector2Scale(dir, t));
    result.normal = n;
    return result;
}