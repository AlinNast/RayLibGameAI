#pragma once
#include <cfloat>
#include <cmath>
#include <vector>
#include "raylib.h"
#include "raymath.h"

// Pure data + math. Knows nothing about steering, so it is reusable.
class Path {
public:
    explicit Path(std::vector<Vector2> pts, bool loop = true) // asumes at least 2 points, if not it doesent work
        : points(std::move(pts)), loop(loop)
    {
        const int lSize = (int)points.size();
        segCount = loop ? lSize : lSize - 1;
        total = 0.0f;
        for (int i = 0; i < segCount; ++i) {
            Vector2 a = points[i], b = points[(i + 1) % lSize];
            float len = Vector2Distance(a, b);
            cumulative.push_back(total);     // distance at the START of segment i
            lengths.push_back(len);
            total += len;
        }
    }

    float Length() const { return total; }

    // "How far along the path (in px) is the point closest to p?"
    float GetParam(Vector2 p) const
    {
        float bestDistSq = FLT_MAX, bestParam = 0.0f;
        const int n = (int)points.size();
        for (int i = 0; i < segCount; ++i) {
            Vector2 a = points[i];
            Vector2 ab = Vector2Subtract(points[(i + 1) % n], a);
            float lenSq = Vector2DotProduct(ab, ab);
            if (lenSq < 0.0001f) continue;                       // skip zero-length segments

            float t = Clamp(Vector2DotProduct(Vector2Subtract(p, a), ab) / lenSq, 0.0f, 1.0f);
            Vector2 closest = Vector2Add(a, Vector2Scale(ab, t));
            float dSq = Vector2DistanceSqr(p, closest);

            if (dSq < bestDistSq) {
                bestDistSq = dSq;
                bestParam = cumulative[i] + t * lengths[i];
            }
        }
        return bestParam;
    }

    // "Which world point is param px along the path?"
    Vector2 GetPosition(float param) const
    {
        if (loop) {
            param = fmodf(param, total);                         // wrap around the loop
            if (param < 0.0f) param += total;
        }
        else {
            param = Clamp(param, 0.0f, total);
        }

        const int n = (int)points.size();
        for (int i = 0; i < segCount; ++i) {
            if (param <= cumulative[i] + lengths[i] || i == segCount - 1) {
                float t = (lengths[i] > 0.0001f) ? (param - cumulative[i]) / lengths[i] : 0.0f;
                return Vector2Lerp(points[i], points[(i + 1) % n], t);
            }
        }
        return points[0];
    }

    void Draw() const
    {
        const int n = (int)points.size();
        for (int i = 0; i < segCount; ++i)
            DrawLineEx(points[i], points[(i + 1) % n], 2.0f, Fade(YELLOW, 0.5f));
        for (const Vector2& p : points)
            DrawCircleV(p, 4.0f, Fade(YELLOW, 0.8f));
    }

private:
    std::vector<Vector2> points;
    std::vector<float>   cumulative;   // distance from the start to the beginning of each segment
    std::vector<float>   lengths;
    float total = 0.0f;
    int   segCount = 0;
    bool  loop;
};