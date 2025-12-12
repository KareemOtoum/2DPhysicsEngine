// main.cpp, created by Andrew Gossen.

#include <iostream>
#include <vector>
#include "core/World.hpp"
#include "core/RigidBody.hpp"
#include "core/Transform.hpp"
#include "visuals/Visuals.hpp"

int main(){

    World world;
    Visuals gfx(world);

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

    int test=10;
    for (int i=0;i<test;i++){
        RigidBody t4(4,1.0,2.0);
        t4.snapTo(Vec2(static_cast<float>(-10+(i*2+3)), 3.0f));   
        t4.colour=Colour{0.0f,255.0f,0.0f};
        t4.rotate(1.5708*1.5f);      
        t4.restitution=0.2f;
        t4.linearVelocity=Vec2(0.0f,0.0f);
        world.getBodies().push_back(t4);
    }

    for (int i=0;i<test;i++){
        RigidBody t4(4,1.0,2.0);
        t4.snapTo(Vec2(static_cast<float>(-150+i+3), 90.0f));   
        t4.colour=Colour{0.0f,255.0f,0.0f};
        t4.rotate(1.5708*1.5f);      
        t4.restitution=0.2f;
        t4.linearVelocity=Vec2(30.0f,0.0f);
        world.getBodies().push_back(t4);
    }


    // Main loop
    gfx.renderLoop();

    return 0;

}
