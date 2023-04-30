#include "impactData.h"

void ImpactData::ProjectionResolve(){    

    float aPushSize = (depth / (a->invMass + b->invMass )) * a->invMass;
    float bPushSize = (depth / (a->invMass + b->invMass )) * b->invMass;
    
    a->position -= collisionNormal * aPushSize;
    b->position += collisionNormal * bPushSize;           
}


void ImpactData::CollisionImpulseResolve(){
    this->ProjectionResolve();    
   
    Body *a = this->a;
    Body *b = this->b;

    // We get the minimum boucinesss between both objects
    float elas = std::min(a->bounce, b->bounce);

    Vec2 ra = this->end - a->position;
    Vec2 rb = this->start - b->position;

    float w = a->angularVelocity;

    Vec2 va = a->velocity + Vec2( -(w * ra.y), (w * ra.x)); 
    Vec2 vb = b->velocity - Vec2( -(w * rb.y), (w * rb.x));

    // We calculate the relative belocity between a and b 
    const Vec2 vrel = va - vb;

    float numerator = -(1 + elas) * vrel.Dot(this->collisionNormal);
    float denominator = (a->invMass + b->invMass);
          denominator += (ra.Cross(this->collisionNormal) * ra.Cross(this->collisionNormal)) * a->invI;
          denominator += (rb.Cross(this->collisionNormal) * rb.Cross(this->collisionNormal)) * b->invI;

    float ImpulseMag = numerator / denominator;

    Vec2 ImpulseDirection = collisionNormal * ImpulseMag;

    Vec2 impulseA = ImpulseDirection * a->invMass;
    //a->ApplyImpulse(impulseA, ra); 
    Vec2 impulseB = ImpulseDirection * b->invMass;
    //b->ApplyImpulse(-impulseB, rb); // Apply Impulse to B
}
