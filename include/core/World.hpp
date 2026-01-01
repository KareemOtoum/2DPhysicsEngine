// World.hpp, created by Andrew Gossen.

// -----
// Owns and simulates all rigid bodies in the physics world.

// Ownership & Lifetime:
// - World owns all RigidBody instances stored in m_bodies.
// - Bodies are stored by value for cache-friendly iteration.
// - I havent safeguarded m_bodies, references/pointers to elements may be invalidated if m_bodies
//   reallocates (e.g., when adding/removing bodies).

// The actual physics simulation :
// - step(dt) advances the simulation by dt seconds
//  this dt value is integrated in the step function to advance the simulation.

// Thread Safety:
// - World is NOT thread-safe.
// - All access must occur from the physics thread.
// -------

#pragma once
#include "core/RigidBody.hpp"
#include <vector>
#include "stats/world_stats.hpp"

class World{ 

    public:

    Vec2 getGravity() const{ return gravity; } 
    std::vector<RigidBody>& getBodies() { return m_bodies; } // Return rigid bodies in the world 
    void step(float dt); // Step function for the world, called after each frame is rendered 
    WorldStats& getStats() { return m_stats; } 

    private:

    std::vector<RigidBody> m_bodies; // All rigid bodies, static and non-static, in the world ( Of which the world takes ownership)
    int solverIterations{10}; // Numver of times collisions are solved per step 
    Vec2 gravity{0.0f,-9.81f}; 
    float m_yBounds=100.0f;
    WorldStats m_stats;

};

// The narrow phase for collision checking, using an expensive but definitive SAT test.
bool narrowPhase(RigidBody& A,RigidBody& B, WorldStats& m_stats); 
