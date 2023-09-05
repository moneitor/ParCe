#include "impactData.h"

void ImpactData::ProjectionResolve(){    

    float aPushSize = (depth / (a->invMass + b->invMass )) * a->invMass;
    float bPushSize = (depth / (a->invMass + b->invMass )) * b->invMass;
    
    a->position -= collisionNormal * aPushSize;
    b->position += collisionNormal * bPushSize;        

    a->shape->UpdateVertices(a->position, a->angle);
    b->shape->UpdateVertices(b->position, b->angle);    
}


void ImpactData::CollisionImpulseResolve(){
    this->ProjectionResolve();          
    float elas = std::min(a->bounce, b->bounce);
    float friction = std::min(a->friction, b->friction);
    
    Vec2 ra = this->end - a->position;
    Vec2 rb = this->start - b->position;

    Vec2 va = a->velocity + Vec2( (-a->angularVelocity * ra.y), (a->angularVelocity * ra.x)); 
    Vec2 vb = b->velocity + Vec2( (-b->angularVelocity * rb.y), (b->angularVelocity * rb.x));

    const Vec2 vrel = va - vb;

    float numeratorNormal = -(1 + elas) * vrel.Dot(collisionNormal);

    float denominatorNormal = (a->invMass + b->invMass)
    + (ra.Cross(collisionNormal) * ra.Cross(collisionNormal)) * a->invI
    + (rb.Cross(collisionNormal) * rb.Cross(collisionNormal)) * b->invI;

    float ImpulseMagNormal = numeratorNormal / denominatorNormal;

    Vec2 impulseNormal = this->collisionNormal * ImpulseMagNormal; 
    
    Vec2 tangent = this->collisionNormal.Normal();  

    float numeratorTangent = friction * -(1 + elas) * vrel.Dot(tangent);

    float denominatorTangent = (a->invMass + b->invMass)
    + (ra.Cross(tangent) * ra.Cross(tangent)) * a->invI
    + (rb.Cross(tangent) * rb.Cross(tangent)) * b->invI;

    float ImpulseMagTangent = numeratorTangent / denominatorTangent;    
        
    Vec2 impulseTangent = tangent * ImpulseMagTangent;

    Vec2 impulse = impulseNormal + impulseTangent;
    a->ApplyImpulseAtPoint(impulse, ra);     
    b->ApplyImpulseAtPoint(-impulse, rb);
}


void ImpactData::CollisionTangentImpulseResolve(){
    this->ProjectionResolve();          
    float elas = std::min(a->bounce, b->bounce);
    float friction = std::min(a->friction, b->friction);
    
    Vec2 ra = this->end - a->position;
    Vec2 rb = this->start - b->position;

    Vec2 va = a->velocity + Vec2( (-a->angularVelocity * ra.y), (a->angularVelocity * ra.x)); 
    Vec2 vb = b->velocity + Vec2( (-b->angularVelocity * rb.y), (b->angularVelocity * rb.x));

    Vec2 tangent = Vec2(collisionNormal.y, -collisionNormal.x).Normalize();

    const Vec2 vrel = va - vb;

    float numeratorTangent = friction * -(1 + elas) * vrel.Dot(tangent);

    float denominatorTangent = (a->invMass + b->invMass)
    + (ra.Cross(tangent) * ra.Cross(tangent)) * a->invI
    + (rb.Cross(tangent) * rb.Cross(tangent)) * b->invI;

    float ImpulseMagTangent = numeratorTangent / denominatorTangent;

    Vec2 impulseTangent = tangent * ImpulseMagTangent;       
    a->ApplyImpulseAtPoint(impulseTangent, ra);     
    b->ApplyImpulseAtPoint(-impulseTangent, rb);
}