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
    const PolygonShape* polygonShape = (PolygonShape*) polygon->shape;
    const CircleShape* circleShape = (CircleShape*) circle->shape;
    const std::vector<Vec2>& polygonVertices = polygonShape->vertices;

    bool isOutside = false;
    Vec2 minCurrVertex;
    Vec2 minNextVertex;
    float distanceCircleEdge = std::numeric_limits<float>::lowest();

    // Loop all the edges of the polygon/box finding the nearest edge to the circle center
    for (int i = 0; i < polygonVertices.size(); i++) {
        int currVertex = i;
        int nextVertex = (i + 1) % polygonVertices.size();
        Vec2 edge = polygonShape->EdgeAt(currVertex);
        Vec2 normal = edge.Normal();

        // Compare the circle center with the rectangle vertex
        Vec2 vertexToCircleCenter = circle->position - polygonVertices[currVertex];
        float projection = vertexToCircleCenter.Dot(normal);

        // If we found a dot product projection that is in the positive/outside side of the normal
        if (projection > 0) {
            // Circle center is outside the polygon
            distanceCircleEdge = projection;
            minCurrVertex = polygonVertices[currVertex];
            minNextVertex = polygonVertices[nextVertex];
            isOutside = true;
            break;
        } else {
            // Circle center is inside the rectangle, find the min edge (the one with the least negative projection)
            if (projection > distanceCircleEdge) {
                distanceCircleEdge = projection;
                minCurrVertex = polygonVertices[currVertex];
                minNextVertex = polygonVertices[nextVertex];
            }
        }
    }

    if (isOutside) {
        ///////////////////////////////////////
        // Check if we are inside region A:
        ///////////////////////////////////////
        Vec2 v1 = circle->position - minCurrVertex; // vector from the nearest vertex to the circle center
        Vec2 v2 = minNextVertex - minCurrVertex; // the nearest edge (from curr vertex to next vertex)
        if (v1.Dot(v2) < 0) {
            // Distance from vertex to circle center is greater than radius... no collision
            if (v1.Magnitude() > circleShape->radius) {
                return false;
            } else {
                // Detected collision in region A:
                impact.a = polygon;
                impact.b = circle;
                impact.depth = circleShape->radius - v1.Magnitude();
                impact.collisionNormal = v1.Normalize();
                impact.start = circle->position + (impact.collisionNormal * -circleShape->radius);
                impact.end = impact.start + (impact.collisionNormal * impact.depth);
            }
        } else {
            ///////////////////////////////////////
            // Check if we are inside region B:
            ///////////////////////////////////////
            v1 = circle->position - minNextVertex; // vector from the next nearest vertex to the circle center
            v2 = minCurrVertex - minNextVertex;   // the nearest edge
            if (v1.Dot(v2) < 0) {
                // Distance from vertex to circle center is greater than radius... no collision
                if (v1.Magnitude() > circleShape->radius) {
                    return false;
                } else {
                    // Detected collision in region B:
                    impact.a = polygon;
                    impact.b = circle;
                    impact.depth = circleShape->radius - v1.Magnitude();
                    impact.collisionNormal = v1.Normalize();
                    impact.start = circle->position + (impact.collisionNormal * -circleShape->radius);
                    impact.end = impact.start + (impact.collisionNormal * impact.depth);
                }
            } else {
                ///////////////////////////////////////
                // We are inside region C:
                ///////////////////////////////////////
                if (distanceCircleEdge > circleShape->radius) {
                    // No collision... Distance between the closest distance and the circle center is greater than the radius.
                    return false;
                } else {
                    // Detected collision in region C:
                    impact.a = polygon;
                    impact.b = circle;
                    impact.depth = circleShape->radius - distanceCircleEdge;
                    impact.collisionNormal = (minNextVertex - minCurrVertex).Normal();
                    impact.start = circle->position - (impact.collisionNormal * circleShape->radius);
                    impact.end = impact.start + (impact.collisionNormal * impact.depth);
                }
            }
        }
    } else {
        // The center of circle is inside the polygon... it is definitely colliding!
        impact.a = polygon;
        impact.b = circle;
        impact.depth = circleShape->radius - distanceCircleEdge;
        impact.collisionNormal = (minNextVertex - minCurrVertex).Normal();
        impact.start = circle->position - (impact.collisionNormal * circleShape->radius);
        impact.end = impact.start + (impact.collisionNormal * impact.depth);
    }

    return true;
}