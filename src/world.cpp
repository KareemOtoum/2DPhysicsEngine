// world.cpp, created by Andrew Gossen.
// Utilises collision functions to actually apply impulses to Rigid Bodies after it has been discovered they're in collision.

#include "core/World.hpp"
#include "collision/Collision.hpp"
#include "core/RigidBody.hpp"
#include "math/Math.hpp"
#include "core/Transform.hpp"
#include "collision/AABB.hpp"
#include <cmath>
#include <iostream>

void broadPhase(std::vector<RigidBody>& bodies){ 
    
    // Broad-phase collision detection.
    // - Generates candidate pairs using AABB overlap (O(n^2) currently).
    // - For each candidate pair, calls narrowPhase(A,B) which may modify velocities/positions.
    // Preconditions:
    // - A.transformedVertices / B.transformedVertices are rebuilt here via physEng::worldSpace().
    // Thread-safety: not thread-safe.

    // First do an AABB test
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {

            RigidBody& A=bodies[i];
            RigidBody& B=bodies[j];

            if (A.isStatic && B.isStatic) continue; // No point in evaluating two static bodies         

            physEng::worldSpace(A);
            physEng::worldSpace(B);

            // Define each bodies' AABB bounding box 
            AABB A_AABB=getAABB(A);
            AABB B_AABB=getAABB(B);

            if (AABBintersection(A_AABB,B_AABB)){ // If the AABB boxes intersect move onto the next stage 
                // Move onto narrow phase, implement partioning later on
                narrowPhase(A,B); // Enter the narrow phase now that we've discerned these two bodies are likely colliding
            }

        }
    }
    
}

void World::step(float dt){ 

    // Advances the simulation by dt seconds.
    // Order: integrate forces -> integrate velocities/positions -> detect/resove collisions.
    // Preconditions:
    // - dt > 0
    // Postconditions:
    // - Body transforms updated and caches invalidated (body.update = true on transform change).

    for (auto& body : m_bodies){
        if (!body.isStatic){

            // Integrator using dt
            body.linearAcceleration = gravity;
            body.linearVelocity += body.linearAcceleration * dt;
            body.position += body.linearVelocity * dt;
            body.rotation += body.angularVelocity*dt;
            body.force = Vec2(0, 0); // Going to implement forces later on 
            body.update = true; // invalidate cached transformedVertices

        }

    }

    broadPhase(m_bodies); // Check the broad phase First 
    // Note, the narrowPahse is automatically called within the broadPhase function.

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
        
        float velAlongNormal = vecMath::dot(relativeVel, manifold.normal);
        if (velAlongNormal > 0.0f) continue;  // If they are already separating along the normal, so the collision is going to resolve on its own

        float rADot=vecMath::dot(rA,normal);
        float rBDot=vecMath::dot(rB,normal);

        // Same scalar impulse magnitude 
        float minRestitiution = std::min(A.restitution, B.restitution); // Variable e 

        float j = -(1.0f + minRestitiution) * velAlongNormal;
        float denominator= (A.inverseMass + B.inverseMass + (rADot*rADot)*A.inverseInertia + (rBDot*rBDot)*B.inverseInertia  );
        j /= denominator;
        j /= static_cast<float>(manifold.contactCount);
        Vec2 impulse=manifold.normal*j;

        // Construct impulse manifold
        impulseManifold rotManifold{impulse,radiusA,radiusB}; 
        impulses.push_back(rotManifold);

    }

    // Apply impulses after impulse for all contact points created 
    for (auto& impulseData : impulses){
        A.linearVelocity-=impulseData.impulse*A.inverseMass;
        B.linearVelocity+=impulseData.impulse*B.inverseMass;
        A.angularVelocity += -vecMath::cross(impulseData.rA, impulseData.impulse) * A.inverseInertia;
        B.angularVelocity += vecMath::cross(impulseData.rB, impulseData.impulse) * B.inverseInertia;
    }

};

void narrowPhase(RigidBody& A, RigidBody& B){ 
    
    // Narrow-phase collision detection and resolution for a candidate body pair.
    //
    // Preconditions:
    // - A and B have passed broad-phase testing.
    // - A.transformedVertices and B.transformedVertices are up-to-date.
    //
    // Effects:
    // - Applies impulse-based collision resolution.
    // - May modify A/B positions via penetration correction.

    Manifold m = SATCollision(A, B); // Apply the SAT test to objectively discern if they are in collision
    if (!m.inCollision) return; // Two objects are not colliding. we can stop here

    resolveCollision(m); // At this point, the two objects are colliding, so we must resolve the collision

    // Apply position correction afterwards to seperate the two objects.

    const float percent = 0.4f;  // Error percentage 
    const float slop = 0.01f; // Precision, based on world distance unit 

    float invMassSum = A.inverseMass+B.inverseMass; // Zero implies two static bodies
    if (invMassSum > 0.f){ 
        // Positional correction if two objects are colliding and one is non-static based on penetration depth 
        float corrMag = std::max(m.penetration - slop, 0.f) / invMassSum * percent;
        Vec2 correction = m.normal * corrMag;
        if (!A.isStatic) { A.position -= correction * A.inverseMass; A.update=true; } // Invalidate cache as position cahnged 
        if (!B.isStatic) { B.position += correction * B.inverseMass; B.update=true; } 
    }

}

