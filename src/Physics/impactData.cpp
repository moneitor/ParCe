#include "impactData.h"

void ImpactData::ProjectionResolve(){    

    float aPushSize = (depth / (a->invMass + b->invMass )) * a->invMass;
    float bPushSize = (depth / (a->invMass + b->invMass )) * b->invMass;
    
    a->position -= collisionNormal * aPushSize;
    b->position += collisionNormal * bPushSize;           
}


void ImpactData::CollisionImpulseResolve(ImpactData &impactData){

    this->ProjectionResolve();

    Body *a = impactData.a;
    Body *b = impactData.b;

    // We get the minimum boucinesss between both objects
    float elas = std::min(a->bounce, b->bounce);

    // We calculate the relative belocity between a and b
    Vec2 vrel = a->velocity - b->velocity;

    // We get the vector that points between both centroids
    Vec2 collisionNormal = (b->position - a->position).Normalize();

    // We store the impulse as a float that we will use later
    float ImpulseMag = -(elas + 1) * vrel.Dot(collisionNormal) / (a->invMass + b->invMass);

    // Finally we take the impulse magnitude times the Collision normal for the final impulse
    Vec2 ImpulseDirection = collisionNormal * ImpulseMag;

    if (!(a->isStatic())){
        a->velocity += ImpulseDirection * a->invMass; // Apply Impulse to A
    }
    if (!(b->isStatic())){
        b->velocity -= ImpulseDirection * b->invMass; // Apply Impulse to B
    }      
}
