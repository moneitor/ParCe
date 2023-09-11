#ifndef BODY_H
#define BODY_H

#include "Vec2.h"
#include "shape.h"

#include "math.h"
#include <algorithm>
#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>


class Body
{
    public:
        Body(const Shape &shape, float x, float y, float mass);
        ~Body();   

        int id;
               
        float area;    
        float bounce;     
        float friction;   
        static int count;

        int __active;
        int __static;

        //Linear entities
        Vec2 position;
        Vec2 velocity;
        Vec2 acceleration;

        float mass; 
        float invMass;

        //Rotation entities
        float angle;  // ang
        float angularVelocity; // w
        float angularAcceleration;  // a

        float I;
        float invI;
                    
        Vec2 netForce;        
        float netTorque;

        //Pointer to a shape object
        Shape *shape = {nullptr}; 

        // Pointer to texture so we can map the shapes
        SDL_Texture *tex_map = nullptr;

        //booleans
        bool isStatic();

        void addForce(const Vec2 &force);    
        void clearForces();

        void addTorque(const float &torque);   
        void clearAngularForces();
        
        void screenCollide(const float widht, const float height, float elasticity);   

        void ApplyImpulseLinear(const Vec2 &impulse);
        void ApplyImpulseAngular(const float impulse);
        void ApplyImpulseAtPoint(const Vec2 &impulse, const Vec2 &radiusVec);


        void IntegrateForces(const float dt);
        void IntegrateVelocities(const float dt);

        void SetTexMap(const char* filepath);

        Vec2 LocalSpaceToWorldSpace(const Vec2 &anchor);
        Vec2 WorldSpaceToLocalSpace(const Vec2 &anchor);
};

#endif