#include "World.h"


World::World(float gravity)
    :gravity{-gravity}
{
    this->gravity = - gravity;
}

World::~World(){
    for (auto body: this->bodies)
    {
        delete(body);
    }
    for (auto constraint: this->constraints)
    {
        delete(constraint);
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
    // Temporary vector of intersection constraints
    std::vector<IntersectionConstraint> intersections;

    for (auto &body : bodies)
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

    // 1 = We first need to integrate the forces A = F/M
    // 2 = Then we need to solve the constraints
    // 3 = And finally we integrate the velocities

    // 1 = We first need to integrate the forces A = F/M
    for(auto &body: this->bodies)
    {
        body->IntegrateForces(dt);
    }

    // Detect collisions between bodies
    for (int i = 0; i <= bodies.size() - 1; i++) {
        for (int j = i + 1; j < bodies.size(); j++) {
            Body* a = bodies[i];
            Body* b = bodies[j];
           
            ImpactData impact;
            if (Collisions::isColliding(a, b, impact)) {
                // impact.CollisionImpulseResolve();   
                // Create a new IntersectionConstraint
                Graphics::DrawCircle(impact.start.x, impact.start.y, 5, 0.0, 0xFF00FFFF);
                Graphics::DrawCircle(impact.end.x, impact.end.y, 5, 2.0, 0xFF00FFFF);

                IntersectionConstraint intersection = IntersectionConstraint(impact.a, impact.b, impact.start, impact.end, impact.collisionNormal);    
                intersections.push_back(intersection); 
            }
        }
    } 

    // 2 = Then we need to solve the constraints
    for (auto &constraint : this->constraints)
    {
        constraint->PreSolve(dt);
    }

    for (auto &intersection : intersections)
    {
        intersection.PreSolve(dt);
    }

    for (int i = 0; i < 30; i++)
    {
        for (auto &constraint : this->constraints)
        {
            constraint->Solve();
        }
        for (auto &intersection : intersections)
        {
            intersection.Solve();
        }
    }
    

    for (auto &constraint : this->constraints)
    {
        constraint->PostSolve();
    }
    for (auto &intersection : intersections)
    {
        intersection.PostSolve();
    }

    // 3 = And finally we integrate the velocities
    for (auto &body: this->bodies)
    {
        body->IntegrateVelocities(dt);
    }

}


void World::CheckCollisions()
{
    
}