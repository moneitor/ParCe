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
	V[2] = a->angularVelocity;
	V[3] = b->velocity.x;
	V[4] = b->velocity.y;
	V[5] = b->angularVelocity;
	return V;
}

JointConstraint::JointConstraint()
    :RBDConstraint(), 
    jacobian(1, 6) ,
    cachedlambda(1),
    bias{0.0f}
{
    this->cachedlambda.Zero();
}

JointConstraint::JointConstraint(Body* a, Body* b, const Vec2& anchorPoint)
    : RBDConstraint(), 
    jacobian(1, 6),
    cachedlambda(1),
    bias{0.0f}
{
    this->a = a;
    this->b = b;
    this->aPoint = a->WorldSpaceToLocalSpace(anchorPoint);
    this->bPoint = b->WorldSpaceToLocalSpace(anchorPoint);
    this->cachedlambda.Zero();

}

void JointConstraint::PreSolve(const float dt)
{
        // Get the anchor point position in world space
    const Vec2 anchorAWorld = a->LocalSpaceToWorldSpace(aPoint);
    const Vec2 anchorBWorld = b->LocalSpaceToWorldSpace(bPoint);

    const Vec2 AanchLocal = anchorAWorld - a->position;
    const Vec2 BanchLocal = anchorBWorld - b->position;

    jacobian.Zero();

    Vec2 v1 = (anchorAWorld - anchorBWorld) * 2.0;
    jacobian.vectors[0][0] = v1.x; // A linear velocity.x
    jacobian.vectors[0][1] = v1.y; // A linear velocity.y

    float v2 = AanchLocal.Cross(anchorAWorld - anchorBWorld) * 2.0;
    jacobian.vectors[0][2] = v2;   // A angular velocity

    Vec2 v3 = (anchorBWorld - anchorAWorld) * 2.0;
    jacobian.vectors[0][3] = v3.x; // B linear velocity.x
    jacobian.vectors[0][4] = v3.y; // B linear velocity.y

    float v4 = BanchLocal.Cross(anchorBWorld - anchorAWorld) * 2.0;
    jacobian.vectors[0][5] = v4;   // B angular velocity


    // initial application of cached lambda
    const MatMN jacobianT = jacobian.Transpose();
    VecN impulses = jacobianT * cachedlambda;

    // Apply the impulses to both A and B
    a->ApplyImpulseLinear(Vec2(impulses[0], impulses[1])); // A linear impulse
    a->ApplyImpulseAngular(impulses[2]);                   // A angular impulse
    b->ApplyImpulseLinear(Vec2(impulses[3], impulses[4])); // B linear impulse
    b->ApplyImpulseAngular(impulses[5]); 

    // bias term or baumgarte 
    float C = (anchorBWorld - anchorAWorld).Dot(anchorBWorld - anchorAWorld);
    C = std::max(0.0f, C - 0.01f);
    const float beta = 0.1f;
    bias = (beta / dt) * C;
}

void JointConstraint::Solve() 
{   
    const VecN V = GetVelocities();
    const MatMN invM = GetInvM(); 

    const MatMN jacobianT = jacobian.Transpose();
    
    // Calculate the numerator
    VecN lambdaNumerator = jacobian * V * -1.0f;  // b
    lambdaNumerator[0] -= bias;
    MatMN lambdaDenominator = jacobian * invM * jacobianT; // A

    // Solve the values of lambda using Ax=b (Gaus-Seidel method)
    VecN lambda = MatMN::SolveGaussSeidel(lambdaDenominator, lambdaNumerator);
    cachedlambda += lambda;

    // Compute the final impulses with direction and magnitude
    VecN impulses = jacobianT * lambda;

    // Apply the impulses to both A and B
    a->ApplyImpulseLinear(Vec2(impulses[0], impulses[1])); // A linear impulse
    a->ApplyImpulseAngular(impulses[2]);                   // A angular impulse
    b->ApplyImpulseLinear(Vec2(impulses[3], impulses[4])); // B linear impulse
    b->ApplyImpulseAngular(impulses[5]);                   // B angular impulse
}

void JointConstraint::PostSolve()
{
}
