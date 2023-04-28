#include "Particle.h"

#include <iostream>

Particle::Particle(float x, float y, float mass)
 :position{Vec2(x,y)}, mass{mass}, radius{1}, area{radius*2}, bounce{1}, id{count}
{       
    if (mass != 0){
        this->invMass = 1.0/mass;
    } else {
        this->invMass = 0.0;
    }

    this->count += 1;          
}

Particle::~Particle(){
    std::cout << "Destructor called. " << std::endl;
}


int Particle::count  = 0;


void Particle::screenCollide(const float width, const float height, float elasticity){
    if (this->position.x - this->radius <= 0) {
        this->position.x = this->radius;
        this->velocity.x *= -elasticity;
    } else if (this->position.x + this->radius >= width) {
        this->position.x = width - this->radius;
        this->velocity.x *= -elasticity;
    }
    if (this->position.y - this->radius <= 0) {
        this->position.y = this->radius;
        this->velocity.y *= -elasticity;
    } else if (this->position.y + this->radius >= height) {
        this->position.y = height - this->radius;
        this->velocity.y *= -elasticity;
    }    
}


void Particle::addForce(const Vec2 &force){
    this ->netForce += force;    
}


void Particle::clearForces(){
    this->netForce = Vec2(0,0);
}


void Particle::integrateParticles(float dt){
    this->acceleration = this->netForce * this->invMass;
    
    //integrate acceleration gives you velocity
    this->velocity += this->acceleration * dt;

    //integrate velocity gives position
    this->position += this->velocity * dt;

    clearForces();
}