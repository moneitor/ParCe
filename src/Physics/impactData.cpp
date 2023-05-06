#include "impactData.h"

void ImpactData::ProjectionResolve(){    

    float aPushSize = (depth / (a->invMass + b->invMass )) * a->invMass;
    float bPushSize = (depth / (a->invMass + b->invMass )) * b->invMass;
    
    a->position -= collisionNormal * aPushSize;
    b->position += collisionNormal * bPushSize;           
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


    // Vec2 colNormSwap = Vec2(collisionNormal.y, -collisionNormal.x).Normalize();   
    
    Vec2 colNormSwap = this->collisionNormal.Normal().Normalize();  

    float numeratorTangent =  friction * -(1 + elas) * vrel.Dot(colNormSwap);

    float denominatorTangent = (a->invMass + b->invMass)
    + (ra.Cross(colNormSwap) * ra.Cross(colNormSwap)) * a->invI
    + (rb.Cross(colNormSwap) * rb.Cross(colNormSwap)) * b->invI;

    float ImpulseMagTangent = numeratorTangent / denominatorTangent;
    
    Vec2 impulseNormal = this->collisionNormal * ImpulseMagNormal;     
    Vec2 impulseTangent = this->collisionNormal * ImpulseMagTangent;

    Vec2 impulse = impulseNormal + impulseTangent;
    a->ApplyImpulse(impulse, ra);     
    b->ApplyImpulse(-impulse, rb);
}


void ImpactData::CollisionTangentImpulseResolve(){
    this->ProjectionResolve();          
    float elas = std::min(a->friction, b->friction);
    
    Vec2 ra = this->end - a->position;
    Vec2 rb = this->start - b->position;

    Vec2 va = a->velocity + Vec2( (-a->angularVelocity * ra.y), (a->angularVelocity * ra.x)); 
    Vec2 vb = b->velocity + Vec2( (-b->angularVelocity * rb.y), (b->angularVelocity * rb.x));

    Vec2 colNormSwap = Vec2(collisionNormal.y, -collisionNormal.x).Normalize();

    const Vec2 vrel = va - vb;

    float numerator = -(1 + elas) * vrel.Dot(colNormSwap);

    float denominator = (a->invMass + b->invMass)
    + (ra.Cross(colNormSwap) * ra.Cross(colNormSwap)) * a->invI
    + (rb.Cross(colNormSwap) * rb.Cross(colNormSwap)) * b->invI;

    float ImpulseMag = numerator / denominator;

    Vec2 impulse = colNormSwap * ImpulseMag;       
    a->ApplyImpulse(impulse, ra);     
    b->ApplyImpulse(-impulse, rb);
}