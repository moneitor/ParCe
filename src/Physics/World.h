#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include "Body.h"
#include "constants.h"
#include "collision.h"
#include "../Graphics.h"


class World{
    public:
        World(float gravity);
        ~World();
        
        void AddBody(Body *body);
        
        std::vector<Body*> &GetBodies();

        void AddForce(const Vec2 force);
        void AddTorque(float torque);

        void Integrate(float dt);

        void CheckCollisions();
    
    private:
        std::vector <Body*> bodies;
        std::vector <Vec2> forces;
        std::vector <float> torques;
        float gravity;

};



#endif