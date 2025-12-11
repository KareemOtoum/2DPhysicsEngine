#include "collision/Collision.hpp"
#include "core/Vector2.hpp"
#include "math/Math.hpp"
#include <vector>
#include <algorithm>
#include "visuals/Visuals.hpp"
#include <iostream>

// Helper functions for SAT 
void projectAxis(const std::vector<Vec2>& vertices,const Vec2& normalAxis,float& max,float& min){ // Projects each vertice onto the normal axis and establishes max and min
    float projection = vecMath::dot(vertices[0], normalAxis);
    min = max = projection; // Establish a baseline 
    for (size_t i=1;i<vertices.size();++i){ // Starting from one as we already established vertices[0] 
        Vec2 vertice=vertices[i];
        float projection=vecMath::dot(vertice,normalAxis);
        if (projection<min){ min=projection; }
        if (projection>max) { max=projection; }
    }
}

Vec2 getMean(const std::vector<Vec2>& vertices){ // Arithmetic sum of vertices, i.e. centre point 
    Vec2 mean{0.0f,0.0f};
    for (auto& vector : vertices){
        mean+=vector;
    }
    return mean/static_cast<float>(vertices.size());
}


struct contactResult{
    Vec2 contact1;
    Vec2 contact2;
    int  contactCount{0};
};

struct ContactCandidate {
    Vec2  point;
    float distSq;
};

contactResult getContactPoints(const RigidBody& A, const RigidBody& B) {

    std::vector<ContactCandidate> candidates;
    candidates.reserve(
        A.transformedVertices.size() * B.transformedVertices.size() * 2
    );

    // Helper function that pushes candidates for 'points of P to edges of Q'
    auto gatherCandidates = [&](const RigidBody& P, const RigidBody& Q) {
        const auto& vertsP = P.transformedVertices;
        const auto& vertsQ = Q.transformedVertices;

        for (const Vec2& vP : vertsP) {
            for (size_t i = 0; i < vertsQ.size(); ++i) {
                Vec2 q1 = vertsQ[i];
                Vec2 q2 = vertsQ[(i + 1) % vertsQ.size()];
                Vec2 contact;
                float d2 = vecMath::pointSegmentDistance(q1, q2, vP, contact);
                candidates.push_back({ contact, d2 });
            }
        }
    };

    // Collect from both directions
    gatherCandidates(A, B);
    gatherCandidates(B, A);

    if (candidates.empty()) {
        return { Vec2(0,0), Vec2(0,0), 0 };
    }

    // 1) Find global minimum distance
    float minDistSq = candidates[0].distSq;
    for (const auto& c : candidates) {
        if (c.distSq < minDistSq) {
            minDistSq = c.distSq;
        }
    }

    // 2) Tolerance for "near-minimum" contacts
    //    (tune this based on your world scale)
    const float eps = 0.0001f; 
    float threshold = minDistSq + eps;

    Vec2 contact1{};
    Vec2 contact2{};
    int  contactCount = 0;

    // 3) Pick first contact
    for (const auto& c : candidates) {
        if (c.distSq <= threshold) {
            contact1 = c.point;
            contactCount = 1;
            break;
        }
    }

    // 4) Pick second contact, distinct from first
    for (const auto& c : candidates) {
        if (c.distSq <= threshold &&
            !vecMath::vecCloselyEqual(contact1, const_cast<Vec2&>(c.point))) {
            contact2 = c.point;
            contactCount = 2;
            break;
        }
    }

    return { contact1, contact2, contactCount };

}

bool SATLoop(const RigidBody& A,const RigidBody& B,float& penetration,Vec2& normal){

    std::vector<Vec2> verticesA=A.transformedVertices;
    std::vector<Vec2> verticesB=B.transformedVertices;

    for (size_t i=0;i<verticesA.size();i++){   // Loops through a polygon's vertices to evaluate each normal axis
       
        Vec2 va=verticesA[i]; 
        Vec2 vb=verticesA[(i+1) % verticesA.size()]; // Wrap around indexing 
        Vec2 edge=vb-va;
        Vec2 normalAxis=Vec2(-edge.y,edge.x); // The axis to test for seperation, in Clockwise winding order 
        normalAxis=normalAxis.normalise();

        float maxA,minA;
        float maxB,minB;

        // Project vertices onto normal axis
        projectAxis(verticesA,normalAxis,maxA,minA);
        projectAxis(verticesB,normalAxis,maxB,minB);
      
        if (maxA <= minB || maxB <= minA) { // A gap was found, so there the two vertices A and B ( / polygons ) are seperated.
            return false;
        }

        // At this point, we know there is overlap ( i.e. for this particlar normal there is no seperation ) 
        float axisDepth=std::min(maxA-minB,maxB-minA);
        if (axisDepth<penetration){
            penetration=axisDepth;
            normal=normalAxis;
        }

    }

    return true;

}

// SAT 
Manifold SATCollision(RigidBody& RigidBodyA,RigidBody& RigidBodyB) { 
    
    // Returns true if two vertices ( Representing a polygon ) intersect using the seperating axis thereom. 
  
    float penetration{1000.0f}; // Will yield as the smallest penetration
    Vec2 normal{0.0f,0.0f}; // Will yield as the normal for the smallest penetration
    bool inCollision{true}; // Whether the two objects are in collision or not

    // Evaluate all edge-normals of the polygons  
    if (!SATLoop(RigidBodyA,RigidBodyB,penetration,normal)) inCollision=false;
    if (!SATLoop(RigidBodyB,RigidBodyA,penetration,normal)) inCollision=false;
    
    contactResult contactData;

    if (inCollision){
       contactData=getContactPoints(RigidBodyA,RigidBodyB);
    }

    if (contactData.contactCount >= 1) {
        DebugDraw::drawContactPoint(contactData.contact1, 0.05f);
    }
    if (contactData.contactCount >= 2) {
        DebugDraw::drawContactPoint(contactData.contact2, 0.05f);
        DebugDraw::drawContactSegment(contactData.contact1, contactData.contact2);
    }

    if (vecMath::dot(normal, RigidBodyB.position - RigidBodyA.position) < 0.0f) {
        normal = normal*-1;  // Ensure the normal always points from a to b to avoid merging objects 
    }

    Manifold manifold{ // Build a manifold to describe the outcome 
        RigidBodyA,
        RigidBodyB,
        normal,
        contactData.contact1,
        contactData.contact2,
        contactData.contactCount,
        penetration,
        inCollision
    };

    return manifold;

}  
