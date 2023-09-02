#pragma once
#include "Body.h"
#include "MatMN.h"

class RBDConstraint
{
public:
    RBDConstraint() = default;
    virtual ~RBDConstraint() = default;

    MatMN GetInvM() const;
    VecN GetVelocities() const; // [va.x va.y wa vb.x vb.y wb]
    
    virtual void Solve() {};

    Body *a;
    Body *b;    

    Vec2 aAnchor;  // This is the anchor in the local space of body a
    Vec2 bAnchor;  // This is the anchor in the local space of body b
};


class JointConstraint : public RBDConstraint
{
public:
    JointConstraint();
    JointConstraint(Body *a, Body *b, const Vec2 &anchor);
    ~JointConstraint();

    virtual void Solve() override;

private:
    MatMN jacobian;
    
};


class IntersectionConstraint: public RBDConstraint
{
    MatMN jacobian;
};
