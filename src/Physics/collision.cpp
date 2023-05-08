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

    if ((aStype == ShapeType::POLYGON || aStype == ShapeType::BOX) && 
        bStype == ShapeType::CIRCLE){
        return Collisions::isCollidingPolygonCircle(a, b, impact);
    }

    if (aStype == ShapeType::CIRCLE &&
        (bStype == ShapeType::POLYGON || bStype == ShapeType::BOX)){
        return Collisions::isCollidingPolygonCircle(b, a, impact);
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



bool Collisions::isCollidingPolygonPolygon(Body *a, Body *b, ImpactData &impact){
    const PolygonShape *sA = (PolygonShape*) a->shape;
    const PolygonShape *sB = (PolygonShape*) b->shape;
    Vec2 aAxis, aPoint;
    Vec2 bAxis, bPoint;

    float ABseparation = sA->FindMinSeparation(sB, aAxis, aPoint);    
    if(ABseparation >= 0){
        return false;
    }

    float BAseparation = sB->FindMinSeparation(sA, bAxis, bPoint);
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
        impact.start = impact.end - (impact.collisionNormal * impact.depth);
    }

    return true;
}

bool Collisions::isCollidingPolygonCircle(Body *polygon, Body *circle, ImpactData &impact){
    const PolygonShape *poly = (PolygonShape*) polygon->shape;
    const std::vector<Vec2> &polyVertices = poly->vertices;

    const CircleShape *bCircle = (CircleShape*) circle->shape;

    float maxProjection = std::numeric_limits<float>::lowest();;

    Vec2 minVert0;
    Vec2 minVert1;

    for(int i=0; i < (polyVertices.size()); i++){      
        int vert0 = i;
        int vert1 = (i + 1) % polyVertices.size();

        Vec2 edge = poly->EdgeAt(i);
        Vec2 normal = edge.Normal();

        Vec2 vertToCircle = circle->position - polyVertices[i];
        float projection = vertToCircle.Dot(normal); 
        

        while (projection > 0 && projection > maxProjection){
            maxProjection = projection;
            minVert0 = poly->vertices[vert0];
            minVert1 = poly->vertices[vert1];
            // break;
        }           
        Graphics::DrawFillCircle(minVert0.x, minVert0.y, 5, 0xFF0000FF); 
        Graphics::DrawFillCircle(minVert1.x, minVert1.y, 5, 0xFF0000FF);   

        
    }
    Vec2 edgedir = minVert1 - minVert0;
    Vec2 vertToCircle2 = circle->position - minVert0;
    float projectionCompare = edgedir.Dot(vertToCircle2);      

    if (projectionCompare <= 0){
        std::cout << "We are in A " << std::endl;
    } 
    if (projectionCompare >= edgedir.Magnitude()){
        std::cout << "We are in B " << std::endl;
    }
    if (projectionCompare > 0 && projectionCompare < edgedir.Magnitude()){
        std::cout << "We are in Center " << std::endl;
    }

    
    return false;
}