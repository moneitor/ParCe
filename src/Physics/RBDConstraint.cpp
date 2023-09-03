#include "RBDConstraint.h"


MatMN RBDConstraint::GetInvM() const
{
    MatMN invM(6, 6);
    invM.Zero();

    invM.vectors[0][0] = a->invMass;
    invM.vectors[1][1] = a->invMass;
    invM.vectors[2][2] = a->invI;

    invM.vectors[3][3] = b->invMass;
    invM.vectors[4][4] = b->invMass;
    invM.vectors[5][5] = b->invI;

    return invM;
}

VecN RBDConstraint::GetVelocities() const
{
    VecN V(6);
    V.Zero();

    V[0] = a->velocity.x;
    V[1] = a->velocity.y;
    V[2] = a->angularVelocity; // radians per second

    V[3] = b->velocity.x;
    V[4] = b->velocity.y;
    V[5] = b->angularVelocity; // radians per second

    return V;

}

JointConstraint::JointConstraint()
:RBDConstraint(),
jacobian(1,6)
{
}

JointConstraint::JointConstraint(Body *a, Body *b, const Vec2 &anchor)
:RBDConstraint(),
jacobian(1,6)
{
    this->a = a;
    this->b = b;
    this->aAnchor = a->WorldSpaceToLocalSpace(anchor);
    this->bAnchor = b->WorldSpaceToLocalSpace(anchor);
}

JointConstraint::~JointConstraint()
{
}

void JointConstraint::Solve()
{
    // Update the Jacobian with the values of the current timestep
    // V = GetVelocities()
    // invM = GetInvM()
    // Calculating lambda -> impulse to apply to objects A and B
    // Apply lambda impulse to the objects

    float bias = 0.0f;

    const Vec2 anchAWorld = a->LocalSpaceToWorldSpace(aAnchor);
    const Vec2 anchBWorld = b->LocalSpaceToWorldSpace(bAnchor);

    const Vec2 anchALocal = anchAWorld - a->position;
    const Vec2 anchBLocal = anchBWorld - b->position;

    Vec2 v1 = (anchAWorld - anchBWorld) * 2;
    jacobian.vectors[0][0] = v1.x; // aBody linear velocity X
    jacobian.vectors[0][1] = v1.y; // aBody linear velocity Y

    float v2 = anchALocal.Cross(anchAWorld - anchBWorld) * 2.0f; 
    jacobian.vectors[0][2] = v2;  // aBody angular velocity

    Vec2 v3 = (anchBWorld - anchAWorld) * 2;
    jacobian.vectors[0][3] = v3.x; // bBody linear velocity X
    jacobian.vectors[0][4] = v3.y; // bBody linear velocity Y

    float v4 = anchBLocal.Cross(anchBWorld - anchAWorld) * 2.0f;
    jacobian.vectors[0][5] = v4; //bBody angular velocity


    const VecN V = GetVelocities();
    const MatMN invM = GetInvM();

    const MatMN jacobianT = jacobian.Transpose();

    VecN lambdaNumerator = ((jacobian * V) + bias) * -1;
    MatMN lambdaDenominator = jacobian * invM * jacobianT;

    VecN lambda = MatMN::SolveGaussSeidel(lambdaDenominator, lambdaNumerator);
}
