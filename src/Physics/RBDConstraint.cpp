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


// INTERSECTION CONSTRAINT

IntersectionConstraint::IntersectionConstraint()
    :RBDConstraint(), 
    jacobian(2, 6) ,
    cachedlambda(2),
    bias{0.0f},
    friction{0.0f}
{
    this->cachedlambda.Zero();
}


IntersectionConstraint::IntersectionConstraint(Body *a, Body *b, Vec2 &aCollision, Vec2 &bCollision, const Vec2 &CollisionNormal)
    : RBDConstraint(), 
    jacobian(2, 6),
    cachedlambda(2),
    normal{CollisionNormal},
    bias{0.0f},
    friction{0.0f}
{
    this->a = a;
    this->b = b;
    this->aPoint = a->WorldSpaceToLocalSpace(aCollision);
    this->bPoint = b->WorldSpaceToLocalSpace(bCollision);
    this->normal = a->WorldSpaceToLocalSpace(normal); // Normals are stored in the local space of object A
    this->cachedlambda.Zero();
}

void IntersectionConstraint::PreSolve(const float dt)
{
        // Get the collisions point position in world space
    const Vec2 pa = a->LocalSpaceToWorldSpace(aPoint);
    const Vec2 pb = b->LocalSpaceToWorldSpace(bPoint);
    Vec2 N = a->LocalSpaceToWorldSpace(this->normal);

    const Vec2 ra = pa - a->position;
    const Vec2 rb = pb - b->position;

    jacobian.Zero();
    // Populate the first row of the jacobian
    Vec2 v1 = -N;
    jacobian.vectors[0][0] = v1.x; // A linear velocity.x
    jacobian.vectors[0][1] = v1.y; // A linear velocity.y

    float v2 = -ra.Cross(N);
    jacobian.vectors[0][2] = v2;   // A angular velocity

    Vec2 v3 = N;
    jacobian.vectors[0][3] = v3.x; // B linear velocity.x
    jacobian.vectors[0][4] = v3.y; // B linear velocity.y

    float v4 = rb.Cross(N);
    jacobian.vectors[0][5] = v4;   // B angular velocity

    // Populate the second row of the jacobian using the tangent vector
    Vec2 tangent = N.Normal();
    
    float friction = std::max(a->friction, b->friction);

    if (friction > 0.0f)
    {
        Vec2 f1 = tangent;

        jacobian.vectors[1][0] = -f1.x; // A linear velocity.x
        jacobian.vectors[1][1] = -f1.y; // A linear velocity.y

        float f2 = -ra.Cross(tangent);
        jacobian.vectors[1][2] = f2;   // A angular velocity

        Vec2 f3 = tangent;
        jacobian.vectors[1][3] = f3.x; // B linear velocity.x
        jacobian.vectors[1][4] = f3.y; // B linear velocity.y

        float f4 = rb.Cross(tangent);
        jacobian.vectors[1][5] = f4;   // B angular velocity
    }

    // initial application of cached lambda
    const MatMN jacobianT = jacobian.Transpose();
    VecN impulses = jacobianT * cachedlambda;

    // Apply the impulses to both A and B
    a->ApplyImpulseLinear(Vec2(impulses[0], impulses[1])); // A linear impulse
    a->ApplyImpulseAngular(impulses[2]);                   // A angular impulse
    b->ApplyImpulseLinear(Vec2(impulses[3], impulses[4])); // B linear impulse
    b->ApplyImpulseAngular(impulses[5]); 

    // bias term or baumgarte 
    float C = (pb - pa).Dot(-N);
    C = std::min(0.0f, C + 0.01f);
    const float beta = 0.1f;


    // Calculation of relative velocity between bodies for bounciness
    Vec2 v_bodyA = a->velocity + Vec2(-a->angularVelocity * ra.y, a->angularVelocity * ra.x);
    Vec2 v_bodyB = b->velocity + Vec2(-b->angularVelocity * rb.y, b->angularVelocity * rb.x);
    float relativeVelocityDifference = (v_bodyA - v_bodyB).Dot(N);

    // Get the minimum bounciness between the two colliding objects
    float epsilon = std::min(a->bounce, b->bounce);

    // add result of epsilon times the relativeVelocityDifference to the bias
    bias = (beta / dt) * C + (epsilon * relativeVelocityDifference);
}

void IntersectionConstraint::Solve()
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

    VecN oldLambda = cachedlambda;

    cachedlambda += lambda;

    cachedlambda[0] = (cachedlambda[0] < 0.0f) ? 0.0f : cachedlambda[0];

    if (this->friction > 0.0f)
    {
        const float maxFriction = cachedlambda[0] * friction;
        cachedlambda[1] = std::clamp(cachedlambda[1], - maxFriction, maxFriction);
    }

    lambda = cachedlambda - oldLambda;

    // Compute the final impulses with direction and magnitude
    VecN impulses = jacobianT * lambda;

    // Apply the impulses to both A and B
    a->ApplyImpulseLinear(Vec2(impulses[0], impulses[1])); // A linear impulse
    a->ApplyImpulseAngular(impulses[2]);                   // A angular impulse
    b->ApplyImpulseLinear(Vec2(impulses[3], impulses[4])); // B linear impulse
    b->ApplyImpulseAngular(impulses[5]);                   // B angular impulse
}

void IntersectionConstraint::PostSolve()
{
}
