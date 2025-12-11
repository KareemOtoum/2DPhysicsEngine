// AABB ( Axis Aligned Bounding Box ), created by Andrew Gossen.

#pragma once
#include "core/Vector2.hpp"
#include "core/RigidBody.hpp"

struct AABB{ // AAB object, with a max and min for easy collision detection
    Vec2 min;
    Vec2 max;
};

AABB getAABB(const RigidBody& Body){

    const Vec2& first = Body.transformedVertices[0];
    Vec2 min = first;
    Vec2 max = first;

    for (size_t i = 1; i < Body.transformedVertices.size(); ++i){
        
        const Vec2& v = Body.transformedVertices[i];

        if (v.x < min.x) min.x = v.x;
        if (v.y < min.y) min.y = v.y;
        if (v.x > max.x) max.x = v.x;
        if (v.y > max.y) max.y = v.y;

    }

    AABB result{
        Vec2(min.x,min.y),
        Vec2(max.x,max.y)
    };

    return result;

}

bool AABBintersection(const AABB& a, const AABB& b) {
    // If one box is completely to the left of the other
    if (a.max.x < b.min.x || b.max.x < a.min.x) return false;
    // If one box is completely above the other
    if (a.max.y < b.min.y || b.max.y < a.min.y) return false;
    // Otherwise, they overlap (including touching edges)
    return true;
} 