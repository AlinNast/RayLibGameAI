#pragma once
#include <array>
#include "raylib.h"
#include "raymath.h"
#include "Agent.h"

struct Formation {
    static constexpr int MaxFollowers = 4;

    Agent* leader = nullptr;                      // the anchor
    std::array<Agent*, MaxFollowers> slots{};     // nullptr = free slot
    float backSpacing = 55.0f;                    // px behind the leader per row
    float sideSpacing = 50.0f;                    // px sideways per row
    float savedLeaderMaxSpeed = 200.0f;           // restored when the formation ends

    // Offset in the leader's local frame: x = forward, y = right.
    //   slot 0,1 -> row 1 (left, right)    slot 2,3 -> row 2 (left, right)
    Vector2 SlotOffset(int slot) const
    {
        int   row = slot / 2 + 1;
        float side = (slot % 2 == 0) ? -1.0f : 1.0f;
        return { -row * backSpacing, side * row * sideSpacing };
    }

    Vector2 SlotWorldPosition(int slot) const
    {
        return Vector2Add(leader->kinematic.position,
            Vector2Rotate(SlotOffset(slot), leader->kinematic.orientation));
    }

    int FirstFreeSlot() const
    {
        for (int i = 0; i < MaxFollowers; ++i)
            if (!slots[i]) return i;
        return -1;
    }

    bool Contains(const Agent* a) const
    {
        if (a == leader) return true;
        for (const Agent* s : slots) if (s == a) return true;
        return false;
    }
};