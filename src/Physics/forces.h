#ifndef FORCES_H
#define FORCES_H

#include "Vec2.h"
#include "Particle.h"
#include "Body.h"

struct Forces{
    
    //static addGravitationalForce();
    static Vec2 DragForce(const Particle &particle, float k, float density=1, float Area=1, float dt=0.016);
    static Vec2 Friction(const Particle &particle, float kDrag=0);    

    static Vec2 DragForceBody(const Body &body, float k, float density=1, float Area=1, float dt=0.016);
    static Vec2 FrictionBody(const Body &body, float kDrag=0); 
};

#endif