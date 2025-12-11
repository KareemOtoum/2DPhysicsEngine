// main.cpp, created by Andrew Gossen.

#include <iostream>
#include <vector>
#include "core/World.hpp"
#include "core/RigidBody.hpp"
#include "core/Transform.hpp"
#include "visuals/Visuals.hpp"

int main(){

    Visuals gfx;
    World world;

    RigidBody floor;
    setBoxVertices(floor, 30.0f, 30.0f);    
    floor.snapTo(Vec2(0.0f, -27.0f));         
    floor.rotate(1.5708f);                  
    floor.colour = Colour{150.0f, 255.0f, 255.0f};
    floor.isStatic=true;
    floor.restitution=1.0f;
    world.getBodies().push_back(floor);

    RigidBody floor2;
    setBoxVertices(floor2, 15.0f, 0.6f);    
    floor2.snapTo(Vec2(10.0f, 0.9f));         
    floor2.rotate(1.5708*0.2f);                  
    floor2.colour = Colour{150.0f, 255.0f, 255.0f};
    floor2.isStatic=true;
    floor2.restitution=1.0f;
    world.getBodies().push_back(floor2);

    RigidBody t3(4,1.0,2.0);
    t3.snapTo(Vec2(0.0f, -2.0f));   
    t3.rotate(1.5708*1.5);      
    t3.restitution=0.0f;
    t3.linearVelocity=Vec2(0.0f,0.0f);
    world.getBodies().push_back(t3);

    RigidBody t4(4,1.0,2.0);
    t4.snapTo(Vec2(0.0f, 0.0f));   
    t4.rotate(1.5708*1.2);      
    t4.restitution=0.0f;
    t4.linearVelocity=Vec2(0.0f,0.0f);
    world.getBodies().push_back(t4);


    // Main loop
    gfx.renderLoop(world);

    return 0;

}
