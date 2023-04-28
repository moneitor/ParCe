#ifndef IMPACTDATA_H
#define IMPACTDATA_H


#include "Vec2.h"
#include "Body.h"

struct ImpactData{
    public:
        ImpactData() = default;
        ~ImpactData() = default;       

        Body *a;
        Body *b;

        Vec2 start;
        Vec2 end;

        Vec2 collisionNormal;
        float depth;

        void ProjectionResolve();   
        void CollisionImpulseResolve(ImpactData &impactData);                
};


#endif