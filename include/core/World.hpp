// World.hpp, created by Andrew Gossen.

// -----
// Owns and simulates all rigid bodies in the physics world.
//
// Ownership & Lifetime:
// - World owns all RigidBody instances stored in m_bodies.
// - Bodies are stored by value for cache-friendly iteration.
// - References/pointers to elements may be invalidated if m_bodies
//   reallocates (e.g., when adding/removing bodies).
//
// Simulation Contract:
// - step(dt) advances the simulation by dt seconds.
//
// Thread Safety:
// - World is NOT thread-safe.
// - All access must occur from the physics thread.
// -------

#pragma once
#include "core/RigidBody.hpp"
#include <vector>

class World{ 

    public:

    Vec2 getGravity() const{ return gravity; } 
    std::vector<RigidBody>& getBodies() { return m_bodies; } // Return rigid bodies in the world 
    void step(float dt); // Step function for the world, called after each frame is rendered 

    private:

    std::vector<RigidBody> m_bodies; // All rigid bodies, static and non-static, in the world ( Of which the world takes ownership)
    int solverIterations{10}; // Numver of times collisions are solved per step 
    Vec2 gravity{0.0f,-9.81f}; 

};

// The narrow phase for collision checking, using an expensive but definitive SAT test.
void narrowPhase(RigidBody& A,RigidBody& B); 
