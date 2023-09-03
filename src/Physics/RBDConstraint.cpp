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

    

}
