#ifndef SHAPE_H
#define SHAPE_H

#include <vector>
#include "Vec2.h"
#include "Edge.h"

#include <iostream>


enum ShapeType{
    CIRCLE,
    POLYGON,
    BOX
};

class Shape
{
    public:
        Shape();
        Shape(const Shape &other);

        virtual ~Shape() = default;

        virtual ShapeType GetType() const = 0;
        virtual Shape *Clone() const = 0;
        virtual float GetMomentOfInertia() const = 0;
};

class CircleShape: public Shape {
    public:
        CircleShape(float const radius);
        CircleShape(const CircleShape &other);

        virtual ~CircleShape();

        float radius;

        virtual ShapeType GetType() const override;
        virtual float GetMomentOfInertia() const override;
        virtual Shape *Clone() const override;
};

class PolygonShape: public Shape {
    public:
        PolygonShape() = default;
        PolygonShape(const std::vector<Vec2> &vertices);
        PolygonShape(const PolygonShape &other);
        virtual ~PolygonShape();

        std::vector<Vec2> restVertices;
        std::vector<Vec2> vertices;   
        std::vector<Edge> edges;     

        virtual ShapeType GetType() const override;
        virtual float GetMomentOfInertia() const override;
        virtual Shape *Clone() const override;
        void UpdateVertices(const Vec2 &pos, float angle);        
        std::vector<Edge> GetEdges();
        Vec2 EdgeAt(int index) const;  
        float FindMinSeparation(const PolygonShape* other, Vec2& axis, Vec2& point) const;

};


class BoxShape: public PolygonShape {
    public:
        BoxShape(float width, float height);
        virtual ~BoxShape();

        float width;
        float height;

        virtual ShapeType GetType() const override;
        virtual float GetMomentOfInertia() const override;
        virtual Shape *Clone() const override;
};


#endif