#include "constraints.h"
#include <stdio.h>

//Overload that receives a particle and an anchor
Vec2 Constraints::Constraint(const Particle &particle, Vec2 anchor, float restlength, float k){
    Vec2 d = particle.position - anchor;

    float delta_l = d.Magnitude() - restlength;
    
    Vec2 dNormalized = d.UnitVector();

    float spring_mag = -k * delta_l;

    Vec2 ConstraintForce = dNormalized * spring_mag;

    printf("X Value is :  %f", particle.position.x);
    printf("    Y Value is :  %f", particle.position.y);

    return ConstraintForce;

}


Vec2 Constraints::Constraint(const Particle &particleA, const Particle &particleB, float restlength, float k){
    Vec2 d = particleA.position - particleB.position;

    float delta_l = d.Magnitude() - restlength;
    
    Vec2 dNormalized = d.UnitVector();

    float spring_mag = -k * delta_l;

    Vec2 ConstraintForce = dNormalized * spring_mag;


    return ConstraintForce;
}


Vec2 Constraints::ConstraintDampened(const Particle &particleA, Vec2 anchor, float restlength, float k, float c){
    Vec2 d = particleA.position - anchor;

    float delta_l = d.Magnitude() - restlength;
    
    Vec2 dNormalized = d.UnitVector();

    float spring_mag = -k * delta_l;
    Vec2 dampening = particleA.velocity * - c;

    Vec2 ConstraintForce = dNormalized * spring_mag;


    return ConstraintForce + dampening;
}


Vec2 Constraints::ConstraintDampened(const Particle &particleA, const Particle &particleB, float restlength, float k, float c){
    Vec2 d = particleA.position - particleB.position;

    float delta_l = d.Magnitude() - restlength;
    
    Vec2 dNormalized = d.UnitVector();

    float spring_mag = -k * delta_l;
    Vec2 dampening = particleA.velocity * - c;

    Vec2 ConstraintForce = dNormalized * spring_mag;


    return ConstraintForce + dampening;
}
