#include "World.h"


World::World(float gravity)
    :gravity{-gravity}
{
    this->gravity = - gravity;
}

World::~World(){
    for (auto body: this->bodies){
        delete(body);
    }
}

void World::AddBody(Body *body){
    this->bodies.push_back(body);
}

void World::AddConstraint(RBDConstraint *constraint)
{
    this->constraints.push_back(constraint);
}

std::vector<RBDConstraint *> &World::GetConstraints()
{
    return this->constraints;
}

std::vector<Body *> &World::GetBodies()
{
    return this->bodies;
}

void World::AddForce(const Vec2 force){
    this->forces.push_back(force);
}

void World::AddTorque(float torque){
    this->torques.push_back(torque);
}

void World::Integrate(float dt){
    for (auto body : bodies)
    {  
        Vec2 weight = Vec2(0, 9.8 * body->mass) * PIXELS_PER_METER;
        body->addForce(weight);
        
        for (auto force: forces)
        {
            body->addForce(force);
        }

        for (auto torque: torques)
        {
            body->addTorque(torque);
        }      
    }  

    for (auto body: bodies){
        body->integrateBody(dt);    
    }

    // Temporary iteration approach to solve collisions
    // for (int iter = 0; iter < 5; iter++){
    //     this->CheckCollisions();
    // }
    CheckCollisions();

}


void World::CheckCollisions(){
    for (int i = 0; i <= bodies.size() - 1; i++) {
            for (int j = i + 1; j < bodies.size(); j++) {
                Body* a = bodies[i];
                Body* b = bodies[j];
                a->isColliding = false;
                b->isColliding = false;
                
                ImpactData impact;
                if (Collisions::isColliding(a, b, impact)) {
                    impact.CollisionImpulseResolve();        
                }
            }
        } 
    }