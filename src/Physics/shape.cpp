#include "shape.h"

Shape::Shape(){

}

Shape::Shape(const Shape &other){}


////////////////////////////
//CIRCLE
CircleShape::CircleShape(float radius)
    :radius{radius}  {    
}

CircleShape::CircleShape(const CircleShape &other)
    :radius{other.radius} {
}

CircleShape::~CircleShape(){
    
}

Shape *CircleShape::Clone() const{
    return new CircleShape(radius);    
}

float CircleShape::GetMomentOfInertia() const {
    return 0.5 * (this->radius * this->radius);    
}

ShapeType CircleShape::GetType() const{
    return ShapeType::CIRCLE;
}



// POLYGON
PolygonShape::PolygonShape(const std::vector<Vec2> &vertices)
    :restVertices{vertices}{ 
    }

PolygonShape::PolygonShape(const PolygonShape &other)
    :restVertices{other.vertices}{
    }

PolygonShape::~PolygonShape(){    
}

Shape *PolygonShape::Clone( ) const{
    return new PolygonShape(this->restVertices);
}

float PolygonShape::GetMomentOfInertia()const{
    return 1/2 ;
}

void PolygonShape::UpdateVertices(const Vec2 &pos, float angle){
    for(int i = 0; i < vertices.size(); i++){
        Vec2 rotatedPos = restVertices.at(i).Rotate(angle);
        vertices.at(i) = rotatedPos + pos;
    }
}


std::vector<Edge> PolygonShape::GetEdges(){
    edges.clear();
    for(int i=0; i < (vertices.size()) -1; i++){        
        edges.push_back(Edge(vertices[i], vertices[i+1]));       
    }
    edges.push_back(Edge(vertices[vertices.size()-1], vertices[0]));
    return edges;    
}

ShapeType PolygonShape::GetType() const{
        return ShapeType::POLYGON;
}

Vec2 PolygonShape::EdgeAt(int index) const {
    int currVertex = index;
    int nextVertex = (index + 1) % this->vertices.size();
    return this->vertices[nextVertex] - this->vertices[currVertex];
}


// BOX
BoxShape::BoxShape(float width, float height)
    :width{width}, height{height} { 
        Vec2 vert0 = Vec2(-width/2.0, -height/2.0);
        Vec2 vert1 = Vec2(width/2.0, -height/2.0);
        Vec2 vert2 = Vec2(width/2.0, height/2.0);
        Vec2 vert3 = Vec2(-width/2.0, height/2.0);

        // Initialization of rest vertices (vertices with the shape at the origin)
        restVertices.push_back(vert0);
        restVertices.push_back(vert1);
        restVertices.push_back(vert2);
        restVertices.push_back(vert3);

        // Initialization of moving vertices (vertices that will be updated by the simulation)
        vertices.push_back(vert0);
        vertices.push_back(vert1);
        vertices.push_back(vert2);
        vertices.push_back(vert3);
}

BoxShape::~BoxShape(){
}

Shape *BoxShape::Clone() const {
    return new BoxShape(this->width, this->height);
}

float BoxShape::GetMomentOfInertia() const {
    float widthSquared = this->width * this->width;
    float heightSquared = this->height * this->height;
    return ((widthSquared + heightSquared) / 12.0);
}

ShapeType BoxShape::GetType() const{
        return ShapeType::BOX;
}