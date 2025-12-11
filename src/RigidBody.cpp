#include "core/RigidBody.hpp"
#include "core/Vector2.hpp"

void setBoxVertices(RigidBody& body, float height, float width) {
    
    float left   = -width  / 2.0f;
    float right  =  left   + width;
    float bottom = -height / 2.0f;
    float top    =  bottom + height;

    body.vertices = {
        Vec2(left,  top),
        Vec2(right, top),
        Vec2(right, bottom),
        Vec2(left,  bottom)
    };
    
}

std::vector<Vec2> generateRegularPolygon(int sides, float radius){

    // Winds clockwise
    
    std::vector<Vec2> verts;
    if (sides < 3) return verts;
    verts.reserve(sides);

    // Angle between consecutive vertices
    const float dTheta = 2.0f * M_PI / static_cast<float>(sides);
    
    // Rotate so one vertex points up
    const float startAngle = -M_PI / 2.0f;

    for (int i = 0; i < sides; ++i){
        float theta = startAngle + i * dTheta;
        float x = radius * std::cos(theta);
        float y = radius * std::sin(theta);
        verts.emplace_back(x, y);
    }

    return verts;
}

float computeRegularPolygonInertia(int sides, float mass, float radius){ 

    if (sides < 3 || mass <= 0.0f) return 0.0f; // Either an invalid polygon or a static object 
    float n = static_cast<float>(sides);
    float angle = 2.0f * M_PI / n;
    float I = (mass * radius * radius / 12.0f) * (3.0f + std::cos(angle));
    return I;

}

float computeInverseMass(float mass, bool isStatic){

    if (isStatic || mass <= 0.0f) return 0.0f;
    return 1.0f / mass;

}

RigidBody::RigidBody(int n,float r,float mass) : sides(n), radius(r), mass(mass) {

    vertices=generateRegularPolygon(n,r);
    inertia=computeRegularPolygonInertia(n,mass,r);
    inverseInertia=1/inertia;
    inverseMass=computeInverseMass(mass,isStatic);
    
}