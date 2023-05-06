#ifndef COLLISION_H
#define COLLISION_H

#include "Body.h"
#include "impactData.h"
#include <iostream>

struct Collisions{
    static bool isColliding(Body *a, Body *b, ImpactData &impact);
    static bool isCollidingCircleCircle(Body *a, Body *b, ImpactData &impact);    
    static bool isCollidingPolygonPolygon(Body *a, Body *b, ImpactData &impact);  
    static bool isCollidingPolygonCircle(Body *polygon, Body *circle, ImpactData &impact);
};

#endif