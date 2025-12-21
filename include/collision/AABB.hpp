// AABB.hpp, created by Andrew Gossen.

// ---------
// Axis-Aligned Bounding Box (AABB) utilities for broad-phase collision detection.

// An AABB is a cuboid that encloses a shape, aligned with the co-ordinate axes.
// Used as a cheap early-out test to evaluate whether two bodies are likely colliding
// This is used before expensive SAT collision checking.

// Contracts:
// - getAABB() requires Body.transformedVertices to be up-to-date (world space).
// - AABBintersection() treats touching edges as intersection.
// -------

#pragma once
#include "core/Vector2.hpp"
#include "core/RigidBody.hpp"

struct AABB{ 
    Vec2 min;
    Vec2 max;
};

// Computes an AABB around a body's cached world-space vertices.
AABB getAABB(const RigidBody& Body){

    // -- 
    // Returns an AABB bounding box for a polygon
    // param Body - The polygon to create a bounding box for ( DOES NOT TAKE OWNERSHIP ) 
    // This is assuming that the polygon has had it's transformed vertices calculated already.
    // -- 

    const Vec2& first = Body.transformedVertices[0];
    Vec2 min = first;
    Vec2 max = first;

    for (auto& v : Body.transformedVertices){
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

// Returns true if two AABBs overlap (including touching edges)
bool AABBintersection(const AABB& a, const AABB& b) {

    // Separating axis tests for axis-aligned boxes.
    if (a.max.x < b.min.x || b.max.x < a.min.x) return false;
    if (a.max.y < b.min.y || b.max.y < a.min.y) return false;
    return true;
    
} 