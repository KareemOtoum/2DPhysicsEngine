
// Transform.hpp, created by Andrew Gossen.

// ------
// Holds utility type for 2D rigid transforms (rotation + translation).
// Used to convert points from local space ( relative to Polygon's COM ) to world space.
// All rotations are in radians.
// ------

#pragma once 
#include "Vector2.hpp"
#include "RigidBody.hpp"

struct Transform{ 

    Vec2 position{0.0f,0.0f}; // Transform position 
    float rotation{0.0f}; // Transform measured in radians  

    Transform()=default;
    Transform(const Vec2& position,float rotation) : position(position), rotation(rotation) {}

    // Changes this transform's position 
    void Translate(const Vec2& translation){ 
        position.x+=translation.x;
        position.y+=translation.y;
    }

     // Changes this Transform's rotation 
    void rotate(float translation){
        rotation+=translation; 
    }

    // Applies this transform to a local-space point, returning world-space.
    Vec2 applyTransform(const Vec2& p) const { 
        float c=std::cos(rotation);
        float s=std::sin(rotation);
        Vec2 rotated(
            p.x * c - p.y * s,
            p.x * s + p.y * c 
        );
        return rotated + position; // Return new transformed vector 
    }

};

namespace physEng{

    inline void worldSpace(RigidBody& body) { 

        // -- 
        // This is used to update a RigidBody's vertices from local space ( relative to it's com ) to world space ( Using the x-y world co-ordinate system)
        // param body - RigidBody to update vertices from local space to world space for, ( DOES NOT TAKE OWNERSHIP ) 
        // -- 

        if (!body.update && !body.transformedVertices.empty()) return;

        Transform t(body.position, body.rotation);

        body.transformedVertices.clear();
        body.transformedVertices.reserve(body.vertices.size());

        for (const Vec2& local : body.vertices) {
            body.transformedVertices.push_back(t.applyTransform(local));
        }

        body.update = false; // Set cache update to false as the transformed vertices are up to date 
        
    }

}; // namespace physEng

