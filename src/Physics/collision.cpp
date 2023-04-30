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


float findMinimumSeparation(const PolygonShape &a, const PolygonShape &b, Vec2 &axis, Vec2 &point){
    float separation = std::numeric_limits<float>::lowest();

    for (int i = 0; i < a.vertices.size(); i++){
        Vec2 vertexA = a.vertices[i];
        Vec2 edgeDir = a.EdgeAt(i).Normal();

        float minSeparation = std::numeric_limits<float>::max();
        Vec2 keepVertexB;
        for (int j = 0; j < b.vertices.size(); j++){
            Vec2 vertexB = b.vertices[j];
            float projectABtoEdgeDir = (vertexB - vertexA).Dot(edgeDir);
            if (projectABtoEdgeDir < minSeparation){
                minSeparation = projectABtoEdgeDir;
                keepVertexB = vertexB;
            }
        }
        if (minSeparation > separation){
            separation = minSeparation;
            axis = a.EdgeAt(i);
            point = keepVertexB;
        }
        separation = std::max(minSeparation, separation);       
        
    }
    return separation;
}


bool Collisions::isCollidingPolygonPolygon(Body *a, Body *b, ImpactData &impact){
    const PolygonShape *sA = (PolygonShape*) a->shape;
    const PolygonShape *sB = (PolygonShape*) b->shape;
    Vec2 aAxis, aPoint;
    Vec2 bAxis, bPoint;

    float ABseparation = findMinimumSeparation(*sA, *sB, aAxis, aPoint);    
    if(ABseparation >= 0){
        return false;
    }

    float BAseparation = findMinimumSeparation(*sB, *sA, bAxis, bPoint);
    if(BAseparation >= 0){
        return false;
    }

    impact.a = a;
    impact.b = b;
    if (ABseparation > BAseparation){
        impact.depth = -ABseparation;
        impact.collisionNormal = aAxis.Normal();
        impact.start = aPoint;
        impact.end = impact.start + (impact.collisionNormal * impact.depth);
    }

    if (BAseparation > ABseparation){
        impact.depth = -BAseparation;
        impact.collisionNormal = bAxis.Normal() * -1;
        impact.end = bPoint;
        impact.start = impact.end + (impact.collisionNormal * impact.depth);
    }

    return true;
}