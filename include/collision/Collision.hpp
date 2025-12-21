
// Collision.hpp, created by Andrew Gossen.

// --------
// Collision detection data structures and narrow-phase queries.

// Ownership & Lifetime:
// - Manifold stores NON-owning references to two RigidBody instances in collision with each other.
// - The referenced bodies must outlive the Manifold.
// - Manifold is intended to be short-lived and used within a single
//   world step.

//  Conventions:
// - The normal is guaranteed to point from A -> B and is unit length
// - penetration is the overlap depth along normal (>= 0 when colliding).
// - 0 <= contactCount <= 2 
// -----

#pragma once 
#include "core/RigidBody.hpp"
#include "core/Vector2.hpp"
#include <vector>

struct Manifold{ 
    RigidBody& A;
    RigidBody& B;
    Vec2 normal{0.0f,0.0f}; // Normal pointing from A to B 
    Vec2 contact1{0.0f,0.0f};
    Vec2 contact2{0.0f,0.0f};
    int contactCount{0};
    float penetration{0.0f};
    bool inCollision{false};
};

// Narrow-phase SAT collision test between two rigid bodies.
// Returns a Manifold containing contact data when colliding.
Manifold SATCollision(RigidBody& RigidBodyA,RigidBody& RigidBodyB);  

