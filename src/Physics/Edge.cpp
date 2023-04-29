#include "Edge.h"

Edge::Edge(const Vec2 &vertexA, const Vec2 &vertexB){
    vertices.push_back(vertexA);
    vertices.push_back(vertexB);
}

Edge::~Edge(){
}

Vec2 Edge::GetEdgeCenter(){
    Vec2 vert0 = this->vertices[0];
    Vec2 vert1 = this->vertices[1];

    Vec2 center = vert0 +  ((vert1 - vert0) * 0.5);

    return center;
}