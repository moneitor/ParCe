#ifndef PARTICLE_H
#define PARTICLE_H

#include "Vec2.h"

class Particle
{
    public:
        Particle(float x, float y, float mass);
        ~Particle();   

        Vec2 position;
        float mass;
        float radius;
        float area;    
        float bounce;
        int id;
        static int count;
        float invMass;

        Vec2 velocity;
        Vec2 acceleration;    
        
        Vec2 netForce;         

        void addForce(const Vec2 &force);    
        void clearForces();
        void screenCollide(const float widht, const float height, float elasticity);    

        void integrateParticles(float dt);
};

#endif