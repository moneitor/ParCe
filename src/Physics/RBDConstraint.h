#pragma once

#include "Body.h"
#include "MatMN.h"
#include "impactData.h"

class RBDConstraint {
    public:
        Body* a;
        Body* b;

        Vec2 aPoint; // The anchor point in A's local space
        Vec2 bPoint; // The anchor point in B's local space

        virtual ~RBDConstraint() = default;

        MatMN GetInvM() const;
        VecN GetVelocities() const;

        virtual void PreSolve(const float dt) {}
        virtual void Solve() {}
        virtual void PostSolve() {}
};

class JointConstraint: public RBDConstraint {
    private:
        MatMN jacobian;
        VecN cachedlambda;
        float bias;
    
    public:
        JointConstraint();
        JointConstraint(Body* a, Body* b, const Vec2& anchorPoint);

        void PreSolve(const float dt) override;
        void Solve() override;
        void PostSolve() override;
};

class IntersectionConstraint: public RBDConstraint {
private:
    float bias;
    Vec2 normal; // Normal of the intersection in BodyA local space
    float friction; 
    MatMN jacobian;
    VecN cachedlambda;

public:
    IntersectionConstraint();
    IntersectionConstraint(Body* a, Body* bVec2, Vec2 &aCollision, Vec2 &bCollision, const Vec2 &CollisionNormal);

    void PreSolve(const float dt) override;
        void Solve() override;
    void PostSolve() override;
};