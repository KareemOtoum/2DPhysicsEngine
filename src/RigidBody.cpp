// RigidBody.cpp, created by Andrew Gossen.
// Handles constructor for the RigidBody.hpp, and defines useful functions to generate RigidBodies. 

#include "core/RigidBody.hpp"
#include "core/Vector2.hpp"

void setBoxVertices(RigidBody& body, float width, float height){

    // Sets local-space vertices for an axis-aligned box (centered at COM).
    // Rebuilds transformedVertices immediately using the body's current position/rotation.
    // Effects: overwrites body.vertices and body.transformedVertices.

    float hw = width  * 0.5f;
    float hh = height * 0.5f;

    // Local-space vertices ccw
    body.vertices.clear();
    body.vertices.reserve(4);

    body.vertices.push_back(Vec2(-hw, -hh));
    body.vertices.push_back(Vec2( hw, -hh));
    body.vertices.push_back(Vec2( hw,  hh));
    body.vertices.push_back(Vec2(-hw,  hh));

    // Compute world-space vertices immediately
    body.transformedVertices.clear();
    body.transformedVertices.reserve(4);

    float c = std::cos(body.rotation);
    float s = std::sin(body.rotation);

    for (const Vec2& v : body.vertices){
        Vec2 rotated(
            v.x * c - v.y * s,
            v.x * s + v.y * c
        );
        // Translate into world space
        body.transformedVertices.push_back(body.position + rotated);
    }

}


std::vector<Vec2> generateRegularPolygon(int n, float r){
        
    // Generates local-space vertices for a regular n-gon of radius r (centered at origin).
    // Returns vertices in CCW order (suitable for SAT / outward normals).
    // Preconditions: n >= 3, r > 0.

    std::vector<Vec2> verts;
    if (n < 3) return verts;
    verts.reserve(n);

    // Angle between consecutive vertices
    const float dTheta = 2.0f * M_PI / static_cast<float>(n);
    // Rotate so one vertex points up
    const float startAngle = -M_PI / 2.0f;

    for (int i = 0; i < n; ++i){ // Iterate through each side, and generate a vertex 
        float theta = startAngle + i * dTheta;
        float x = r * std::cos(theta);
        float y = r * std::sin(theta);
        verts.emplace_back(x, y); // Construct the Vector2 point of this vertex within the verts vector 
    }

    return verts;

}

float computeRegularPolygonInertia(int n, float m, float r){ 

    // Computes moment of inertia about the COM for a solid regular n-gon (approx/closed-form).
    // Returns 0 for invalid input or non-dynamic bodies (m <= 0 or n < 3).
    // Units: inertia in (mass * length^2).

    if (n < 3 || m <= 0.0f) return 0.0f; // Either an invalid polygon or a static object 
    n = static_cast<float>(n);
    float angle = 2.0f * M_PI / n;
    float I = (m * r * r / 12.0f) * (3.0f + std::cos(angle));
    return I;

}

float computeInverseMass(float mass, bool isStatic){

    // ---
    // Computes the inverse mass, used to avoid division by zero 
    // param mass - Mass to inverse 
    // param isStatic - Whether the mass of the object we're calculating the inverse mass for is static or not
    // Note : static implies 0 inverse mass 
    // -- 

    if (isStatic || mass <= 0.0f) return 0.0f;
    return 1.0f / mass;

}

RigidBody::RigidBody(int n,float r,float m) : sides(n), radius(r), mass(m) {

    // RigidBody constructor
    // Sets inverse mass for impulse math. Static bodies and non-positive masses return 0.
    // Avoids division by zero and encodes immovable bodies via invMass = 0.

    vertices=generateRegularPolygon(n,r);
    inertia=computeRegularPolygonInertia(n,m,r);
    inverseInertia=1/inertia;
    inverseMass=computeInverseMass(m,isStatic);

}