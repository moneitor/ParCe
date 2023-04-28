#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include "Vec2.h"
#include "Particle.h"


struct Constraints{
    static Vec2 Constraint(const Particle &particle, Vec2 anchor, float restlength, float k);
    static Vec2 Constraint(const Particle &particleA, const Particle &particleB, float restlength, float k);
    static Vec2 ConstraintDampened(const Particle &particleA, const Particle &particleB, float restlength, float k, float c);
    static Vec2 ConstraintDampened(const Particle &particleA, Vec2 anchor, float restlength, float k, float c);
    
};

#endif