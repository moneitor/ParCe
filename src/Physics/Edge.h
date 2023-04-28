#ifndef EDGE_H
#define EDGE_H

#include <vector>
#include "Vec2.h"


struct Edge {
    Edge(const Vec2 &vertexA, const Vec2 &vertexB);
    ~Edge();
    std::vector<Vec2>vertices;

    Vec2 GetEdgeCenter();
};

#endif