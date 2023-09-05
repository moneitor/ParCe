#include "Body.h"
#include "../Graphics.h"



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
    this->shape->UpdateVertices(this->position, this->angle);
    this->count += 1;          
}

Body::~Body(){
    delete this->shape;
    SDL_DestroyTexture(this->tex_map);
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

void Body::ApplyImpulseLinear(const Vec2& j) 
{
    if (isStatic())
        return;
    velocity += j * invMass;
}

void Body::ApplyImpulseAngular(const float j) 
{
    if (isStatic())
        return;
    angularVelocity += j * invI;
}

void Body::ApplyImpulseAtPoint(const Vec2& j, const Vec2& r)
{
    if (isStatic())
        return;
    velocity += j * invMass;
    angularVelocity += r.Cross(j) * invI;
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



void Body::IntegrateForces(const float dt)
{
    if(isStatic())
    {
        return;
    }

    this->acceleration = this->netForce * invMass;
    this->velocity += this->acceleration * dt;    

    this->angularAcceleration = this->netTorque * invI;
    this->angularVelocity += this->angularAcceleration * dt;


    clearForces();
    clearAngularForces();
}

void Body::IntegrateVelocities(const float dt)
{
    if(isStatic())
    {
        return;
    }
    this->position += this->velocity * dt;
    this->angle += this->angularVelocity * dt;

    this->shape->UpdateVertices(this->position, this->angle);
}


void Body::SetTexMap(const char* filepath){
    SDL_Surface *map = IMG_Load(filepath);
    this->tex_map = SDL_CreateTextureFromSurface(Graphics::renderer, map);
    SDL_FreeSurface(map);
}


Vec2 Body::LocalSpaceToWorldSpace(const Vec2& point)
{
    Vec2 rotated = point.Rotate(angle);
	return rotated + position;
}

Vec2 Body::WorldSpaceToLocalSpace(const Vec2& point) 
{
    float translatedX = point.x - position.x;
    float translatedY = point.y - position.y;
    float rotatedX = cos(-angle) * translatedX - sin(-angle) * translatedY;
    float rotatedY = cos(-angle) * translatedY + sin(-angle) * translatedX;
	return Vec2(rotatedX, rotatedY);
}