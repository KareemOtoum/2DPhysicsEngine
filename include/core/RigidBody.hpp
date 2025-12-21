
// RigidBody.h, created by Andrew Gossen.

// ---
// Defines a physics data object representing a single rigid body.

// Ownership & Lifetime:
// - All RigidBody instances are owned by the World class.
// - A RigidBody object does not own external resources.

// Invariants for a RigidBody:
// - vertices are defined in local space relative to the body's COM.
// - position and rotation define the authoritative world transform (i.e. the world-space position/rotation )
// - transformedVertices are cached world-space vertices (world-space vertices represent 
//.  vertices in their actual location, rather than being relative to the COM )
// - If update == true, this implies the body has moved, transformedVertices 
//.  MUST be recached to reflect the body's new position and rotation.

// Thread Safety:
// - RigidBody is NOT thread-safe.
// - Instances must only be accessed and mutated from a single thread
//   (the physics simulation thread).
// --

#pragma once
#include "core/Vector2.hpp"
#include <vector>

enum ShapeType{ // Implement later for optimisation
    Circle,Rectangle,Polygon
};
struct Colour{
    float r,g,b;
};

struct RigidBody{ 

    ShapeType shape; // Used to discern circle or rectangle for more efficent collision detection later on
    int sides; // Sides 
    int radius; // Radius 
    // Constructor 
    RigidBody()=default;
    RigidBody(int n, float radius,float mass);

    Vec2 force;
    Vec2 position{0.0f,0.0f};
    float rotation{0.0f}; // Radians
    Vec2 linearVelocity{0.0f,0.0f};
    Vec2 linearAcceleration{0.0f,0.0f};
    float angularVelocity{0.0f};
    float angularAcceleration{0.0f};
    Colour colour{255.0f,255.0f,255.0f};

    float inertia{0.0f};
    float inverseInertia{0.0f};
    float staticFriction{0.2f};
    float dynamicFriction{0.8f};
    float density{0.0f}; 
    float mass{0.0f};
    float inverseMass{0.0f};
    float restitution{0.0f};
    float area{0.0f};
    bool isStatic{false};
   
    std::vector<Vec2> vertices {}; // Vertices relative to the bodies COM
    std::vector<Vec2> transformedVertices {}; // Cached transformed vertices 
    bool update{false}; // Whether the transformed vertices need to be recalculated 

    // Position and rotation incrementing and setting 

    void move(const Vec2& amount){ // Move RigidBooy by amount
        position+=amount;
        update=true;
    }
    
    void rotate(const float radians){ // Rotate rigid body by given radians 
        rotation+=radians;
        update=true;
    }

    void snapTo(const Vec2& pos){ // Set RigidBody's position to pos 
        position=pos;
        update=true;
    }

};

// Defined in RigidBody.cpp
float calculateInertia(RigidBody& body);
void setBoxVertices(RigidBody& body, float height, float width);
