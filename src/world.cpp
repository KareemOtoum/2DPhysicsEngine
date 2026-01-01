// world.cpp, created by Andrew Gossen.
// Utilises collision functions to actually apply impulses to Rigid Bodies after it has been discovered they're in collision.

#include "core/World.hpp"
#include "collision/Collision.hpp"
#include "core/RigidBody.hpp"
#include "math/Math.hpp"
#include "core/Transform.hpp"
#include "collision/AABB.hpp"
#include "collision/Partitioning.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

std::pair<bool,bool> broadPhase(std::vector<RigidBody>& bodies,WorldStats& m_stats){ 
    
    // Broad-phase collision detection, returns whether the narrow phase was reached.
    // - Generates close candidate pairs (i,j) using AABBS and spatial partioning, where i and j are close in world-space.
    // - For each candidate pair, it is ensured their AABB overlaps, in which the narrow phase is then called for the candidate pair. 
    // Preconditions:
    // - A.transformedVertices / B.transformedVertices are rebuilt here via physEng::worldSpace().
    // Thread-safety: not thread-safe, run from physics thread only.

    bool narrowReached=false;
    bool inCollision=false;

    std::vector<AABB> aabbs;
    aabbs.reserve(bodies.size()); 

    for (auto& body : bodies){ 
        // update world space vertices for bodies still in bounds 

        physEng::worldSpace(body); // Update each body from it's local space vertices to world space 
        AABB box=getAABB(body); // Construct it's AABB
        aabbs.push_back(box);

    }

    partioning::GridConfig gridConfig;
    auto pairs = partioning::buildPairsFromAABBs(aabbs, gridConfig); // Get canditate pairs which are close to each other in world-space 

    for (auto [i,j] : pairs) { // Go through each canditate pair, i.e. i and j are close 
       
        m_stats.broadChecks++;

        RigidBody& A = bodies[i];
        RigidBody& B = bodies[j];

        if (A.isStatic && B.isStatic) continue;

        // Do a final cheap check to ensure their AABBS are overlapping before running an SAT test 
        if (!AABBintersection(aabbs[i], aabbs[j])) continue;
        // At this point, it is very likely they are in collision, so we can run expensive SAT tests
        narrowPhase(A, B, m_stats);
        m_stats.narrowChecks++;

    }

    return {narrowReached,inCollision};
}

void World::step(float dt){ 

    // Advances the simulation by dt seconds.
    // Order: integrate forces -> integrate velocities/positions -> detect/resove collisions.
    // Assumed:
    // - dt > 0
    // Postconditions:
    // - Body transforms updated and caches invalidated (body.update = true on transform change).

    // Integrate
    for (auto& body : m_bodies) {
        if (!body.isStatic) {

            body.linearAcceleration = gravity;
            body.linearVelocity += body.linearAcceleration * dt;
            body.position += body.linearVelocity * dt;
            body.rotation += body.angularVelocity * dt;
            body.force = Vec2(0, 0);
            body.update = true;
            m_stats.bodyUpdates++;
        }
    }

    // Cull out-of-bounds bodies
    m_bodies.erase(
        std::remove_if(m_bodies.begin(), m_bodies.end(),
            [&](const RigidBody& body) {
                return body.position.y < -m_yBounds;
            }),
        m_bodies.end()
    );

    for (int i = 0; i < solverIterations; ++i) {
        auto [narrowPhaseReached,colliding]=broadPhase(m_bodies,m_stats);
        m_stats.narrowChecks+=(int)narrowPhaseReached;
        m_stats.contactsResolved+=(int)colliding;
    }

    m_stats.steps++;

}

struct impulseManifold{ // Used to store impulses to apply all impulses only once all contact points are accounted for 
    Vec2 impulse;
    Vec2 rA;
    Vec2 rB;
};

void resolveCollision(Manifold& manifold){

    // Resolves collision by applying impulses at each contact point.
    // Preconditions:
    // - manifold.inCollision == true
    // - manifold.normal is unit length and points from A -> B
    // - contactCount in [1,2] and contact points are valid
    // Effects:
    // - Modifies A/B linearVelocity and angularVelocity.

    RigidBody& A=manifold.A;
    RigidBody& B=manifold.B;
    const Vec2 normal=manifold.normal;

    std::vector<Vec2> contacts;
    contacts.reserve(manifold.contactCount);
    if (manifold.contactCount >= 1) contacts.push_back(manifold.contact1);
    if (manifold.contactCount >= 2) contacts.push_back(manifold.contact2);

    std::vector<impulseManifold> impulses;

    impulses.reserve(contacts.size());

    float staticFriction=std::min(A.staticFriction,B.staticFriction);
    float dynamicFriction=std::min(A.dynamicFriction,B.dynamicFriction);

    for (auto& contact : contacts){ // Create impulse for each contact point 

        Vec2 radiusA=contact-A.position;
        Vec2 radiusB=contact-B.position;

        // Perpendicular radii
        Vec2 rA=Vec2(-radiusA.y,radiusA.x);
        Vec2 rB=Vec2(-radiusB.y,radiusB.x);

        Vec2 AtangentialVelocity=rA*A.angularVelocity;
        Vec2 BtangentialVelocity=rB*B.angularVelocity;

        Vec2 relativeVel= (
            (B.linearVelocity+BtangentialVelocity)-
            (A.linearVelocity+AtangentialVelocity)
        );

        Vec2 tangent=relativeVel-normal*vecMath::dot(relativeVel,normal);
        
        float velAlongNormal = vecMath::dot(relativeVel, manifold.normal);
        if (velAlongNormal > 0.0f) continue;  // If they are already separating along the normal, so the collision is going to resolve on its own

        bool applyFriction=true;
        
        if (vecMath::floatCloselyEqual(tangent.length(),0)){ // Allow box to microsettle ( stay flat once all velocity is lost )
            applyFriction=false;  
        } else { 
            tangent=tangent.normalise();
        }

        float rADot=vecMath::dot(rA,normal);
        float rBDot=vecMath::dot(rB,normal);
        float minRestitiution = std::min(A.restitution,B.restitution); // Variable e 

        float denominator= (A.inverseMass + B.inverseMass + (rADot*rADot)*A.inverseInertia + (rBDot*rBDot)*B.inverseInertia  ); 
        float j = -(1.0f + minRestitiution) * velAlongNormal;
        j /= denominator;
        j /= static_cast<float>(manifold.contactCount);

        // Rotational and linear manifold 
        Vec2 impulse=manifold.normal*j;
        impulseManifold rotManifold{impulse,radiusA,radiusB}; 
        impulses.push_back(rotManifold);

        // Friction manifold 
        if (applyFriction){

            float rADotTangential=vecMath::dot(rA,tangent);
            float rBDotTangential=vecMath::dot(rB,tangent);

            float denominatorTangential= (
                A.inverseMass + B.inverseMass + 
                (rADotTangential*rADotTangential)*A.inverseInertia + 
                (rBDotTangential*rBDotTangential)*B.inverseInertia
            );

            float jTangent = -vecMath::dot(relativeVel, tangent);;
            jTangent /= denominatorTangential;
            jTangent /= static_cast<float>(manifold.contactCount);

            Vec2 frictionImpulse;

            if (std::abs(jTangent) <= j*staticFriction){
                frictionImpulse=tangent*jTangent;
            } else { 
                frictionImpulse=tangent*-1*j*dynamicFriction;
            }

            impulseManifold frictionManifold{frictionImpulse,radiusA,radiusB}; 
            impulses.push_back(frictionManifold);
        }


    }

    // Apply impulses after impulse for all contact points created 
    for (auto& impulseData : impulses){

        A.linearVelocity-=impulseData.impulse*A.inverseMass;
        B.linearVelocity+=impulseData.impulse*B.inverseMass;
        A.angularVelocity += -vecMath::cross(impulseData.rA, impulseData.impulse) * A.inverseInertia;
        B.angularVelocity += vecMath::cross(impulseData.rB, impulseData.impulse) * B.inverseInertia;
    }


};


bool narrowPhase(RigidBody& A, RigidBody& B,WorldStats& m_stats){  
    
    // Narrow-phase collision detection and resolution for a candidate body pair, return whether a collision was resolved 
    // ( i.e. whether there was actually a collision)
    //
    // Preconditions:
    // - A and B have passed broad-phase testing.
    // - A.transformedVertices and B.transformedVertices are up-to-date.
    //
    // Effects:
    // - Applies impulse-based collision resolution.
    // - May modify A/B positions via penetration correction.

    Manifold m = SATCollision(A, B); // Apply the SAT test to objectively discern if they are in collision
    if (!m.inCollision) return false; // Two objects are not colliding. we can stop here

    resolveCollision(m); // At this point, the two objects are colliding, so we must resolve the collision
    m_stats.contactsResolved++;

    // Apply position correction afterwards to seperate the two objects.

    const float percent = 0.8f;  // Error percentage 
    const float slop = 0.01f; // Precision, based on world distance unit 

    float invMassSum = A.inverseMass+B.inverseMass; // Zero implies two static bodies
    if (invMassSum > 0.f){ 
        // Positional correction if two objects are colliding and one is non-static based on penetration depth 
        float corrMag = std::max(m.penetration - slop, 0.f) / invMassSum * percent;
        Vec2 correction = m.normal * corrMag;
        if (!A.isStatic) { A.position -= correction * A.inverseMass; A.update=true; } // Invalidate cache as position cahnged 
        if (!B.isStatic) { B.position += correction * B.inverseMass; B.update=true; } 
    }

    return true;

}

