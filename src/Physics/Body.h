#ifndef BODY_H
#define BODY_H

#include "Vec2.h"
#include "shape.h"

#include "math.h"
#include <algorithm>
#include <iostream>



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
        const Shape *shape = {nullptr}; 

        //booleans
        bool isColliding;

        bool isStatic();

        void addForce(const Vec2 &force);    
        void clearForces();

        void addTorque(const float &torque);   
        void clearAngularForces();
        
        void screenCollide(const float widht, const float height, float elasticity);   

        void ApplyImpulse(const Vec2 &impulse);
        void ApplyImpulse(const Vec2 &impulse, const Vec2 &radiusVec);

        void integrateLinear(float dt);
        void integrateAngular(float dt);

        void integrateBody(float dt);
};

#endif