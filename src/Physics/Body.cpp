#include "Body.h"


Body::Body(const Shape &shape, float x, float y, float mass){
    this->shape = shape.Clone();    

    this->__active = 1;
    this->__static = 0;

    this->area = 1; //default
    this->position = Vec2(x, y);
    this->velocity = Vec2(0,0);
    this->acceleration = Vec2(0,0);
    this->netForce = Vec2(0,0);

    this->angle = 0;
    this->angularVelocity = 0;
    this->angularAcceleration = 0;
    this->netTorque = 0;

    this->mass = mass;        
    this->bounce = 1;//default
    this->friction = 0.1; // default
    this->id = this->count;    

    if (this->mass != 0){
        this->invMass = 1.0/this->mass;        
    } else {
        this->invMass = 0.0;
    }

    this->isColliding = false;

    I = shape.GetMomentOfInertia() * this->mass; // Final inertia tensor

    if (this->I != 0){
        this->invI = 1.0/this->I;
    } else {
        this->invI = 0;
    }

    this->count += 1;          
}

Body::~Body(){
    delete this->shape;
    std::cout << "Destructor called. " << std::endl;
}


int Body::count  = 0;


void Body::screenCollide(const float width, const float height, float elasticity){
    if (this->shape->GetType() == ShapeType::CIRCLE){
        CircleShape *circleShape = (CircleShape*)this->shape;
        if (this->position.x - circleShape->radius <= 0) {
            this->position.x = circleShape->radius;
            this->velocity.x *= -elasticity;
        } else if (this->position.x + circleShape->radius >= width) {
            this->position.x = width - circleShape->radius;
            this->velocity.x *= -elasticity;
        }
        if (this->position.y - circleShape->radius <= 0) {
            this->position.y = circleShape->radius;
            this->velocity.y *= -elasticity;
        } else if (this->position.y + circleShape->radius >= height) {
            this->position.y = height - circleShape->radius;
            this->velocity.y *= -elasticity;
        }    
    }
}


void Body::addForce(const Vec2 &force){
    this ->netForce += force;    
}


void Body::clearForces(){
    this->netForce = Vec2(0,0);
}


void Body::addTorque(const float &torque){
    this->netTorque += torque;    
}

void Body::clearAngularForces(){
    this->netTorque = 0;
}

bool Body::isStatic(){
    if (fabs((this->invMass - 0.0) < 0.005F)){
        this->__static = 1;
        this->__active = 0;
        return true;
    }
    return false;
}


void Body::ApplyImpulse(const Vec2 &impulse){
    this->velocity += impulse * this->invMass;
}


void Body::ApplyImpulse(const Vec2 &impulse, const Vec2 &radiusVec){
    this->velocity += impulse * this->invMass;
    this->angularVelocity += radiusVec.Cross(impulse) * this->invI; // The cross product is returning a float since this is calculated in 2d
}


void Body::integrateLinear(float dt){
    this->acceleration = this->netForce * this->invMass;
    
    //integrate acceleration gives you velocity
    this->velocity += this->acceleration * dt;

    //integrate velocity gives position
    this->position += this->velocity * dt;

    clearForces();
}

void Body::integrateAngular(float dt){
    if (this->isStatic()){
        return;
    }

    this->angularAcceleration = this->netTorque * invI;
    
    //integrate acceleration gives you velocity
    this->angularVelocity += this->angularAcceleration * dt;

    //integrate velocity gives position
    this->angle += this->angularVelocity * dt;

    clearAngularForces();
}

void Body::integrateBody(float dt){
    if (this->isStatic()){
        //return; Need to do some better shit with this
    }
    
    this->integrateLinear(dt);
    this->integrateAngular(dt);

    if (this->shape->GetType() != ShapeType::CIRCLE){
        PolygonShape *poly = (PolygonShape*)this->shape;
        poly->UpdateVertices(this->position, this->angle);
    }
}