#include "collision.h"
#include <limits>




bool Collisions::isColliding(Body *a, Body *b, ImpactData &impact){
    ShapeType aStype = a->shape->GetType();
    ShapeType bStype = b->shape->GetType();

    if (aStype == ShapeType::CIRCLE && bStype == ShapeType::CIRCLE){
        return Collisions::isCollidingCircleCircle(a, b, impact);
    } 
    
    if ((aStype == ShapeType::POLYGON || aStype == ShapeType::BOX) && 
        (bStype == ShapeType::POLYGON || bStype == ShapeType::BOX)){
        return Collisions::isCollidingPolygonPolygon(a, b, impact);
    }

    return false;
}


bool Collisions::isCollidingCircleCircle(Body *a, Body *b, ImpactData &impact){
    CircleShape *aCircle = (CircleShape*)a->shape;
    CircleShape *bCircle = (CircleShape*)b->shape;

    float aRadius = aCircle->radius;
    float bRadius = bCircle->radius;

    Vec2 aPosition = a->position;
    Vec2 bPosition = b->position;

    float distanceSquared = (bPosition - aPosition).MagnitudeSquared();

    float addedRadius = aRadius + bRadius;

    bool isCollision = distanceSquared <= (addedRadius * addedRadius);

    // Populating impactData
    if (!isCollision)
    {
        std::cout << "NOT COLLIDING:  "<< std::endl;
        return false;
    }
     else 
    { 
        impact.start = bPosition - ((bPosition - aPosition).UnitVector() * bRadius);
        impact.end = aPosition + ((bPosition - aPosition).UnitVector() * aRadius);

        impact.a = a;
        impact.b = b;

        impact.collisionNormal = (bPosition - aPosition).UnitVector();
        impact.depth = (impact.start - impact.end).Magnitude();
    }
    std::cout << "IT IS COLLIDING:  "<< std::endl;
    return true;
}


float findMinimumSeparation(const PolygonShape &a, const PolygonShape &b){
    float separation = std::numeric_limits<float>::lowest();

    for (int i = 0; i < a.vertices.size(); i++){
        Vec2 vertexA = a.vertices[i];
        Vec2 edgeDir = a.EdgeAt(i).Normal();

        float minSeparation = std::numeric_limits<float>::max();
        for (int j = 0; j < b.vertices.size(); j++){
            Vec2 vertexB = b.vertices[j];
            Vec2 vertexAB = (vertexB - vertexA);

            float projectABtoEdgeDir = vertexAB.Dot(edgeDir);
            minSeparation = std::min(projectABtoEdgeDir, minSeparation);
        }
        
        separation = std::max(minSeparation, separation);       
        
    }
    return separation;
}


bool Collisions::isCollidingPolygonPolygon(Body *a, Body *b, ImpactData &impact){
    const PolygonShape *sA = (PolygonShape*) a->shape;
    const PolygonShape *sB = (PolygonShape*) b->shape;
    if(findMinimumSeparation(*sA, *sB) >= 0){
        return false;
    }
    if(findMinimumSeparation(*sB, *sA) >= 0){
        return false;
    }
    return true;
}