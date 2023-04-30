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
    
    Vec2 ra = this->end - a->position;
    Vec2 rb = this->start - b->position;

    Vec2 va = a->velocity + Vec2( (-a->angularVelocity * ra.y), (a->angularVelocity * ra.x)); 
    Vec2 vb = b->velocity + Vec2( (-b->angularVelocity * rb.y), (b->angularVelocity * rb.x));

    const Vec2 vrel = va - vb;

    float numerator = -(1 + elas) * vrel.Dot(collisionNormal);

    float denominator = (a->invMass + b->invMass)
    + (ra.Cross(collisionNormal) * ra.Cross(collisionNormal)) * a->invI
    + (rb.Cross(collisionNormal) * rb.Cross(collisionNormal)) * b->invI;

    float ImpulseMag = numerator / denominator;

    Vec2 impulse = this->collisionNormal * ImpulseMag;       
    a->ApplyImpulse(impulse, ra);     
    b->ApplyImpulse(-impulse, rb);
}
