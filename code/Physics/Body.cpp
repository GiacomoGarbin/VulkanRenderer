//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
m_position( 0.0f ),
m_orientation( 0.0f, 0.0f, 0.0f, 1.0f ),
m_shape( NULL )
{
}

void Body::ApplyImpulseLinear(const Vec3& impulse)
{
	if (m_invMass == 0.0f)
	{
		return;
	}

	m_linearVelocity += impulse * m_invMass;
}

void Body::ApplyImpulseAngular(const Vec3& impulse)
{
	if (m_invMass == 0.0f)
	{
		return;
	}

	m_angularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

	const float maxAngularSpeed = 30.0f; // 30 rad/s
	if (m_angularVelocity.GetLengthSqr() > maxAngularSpeed * maxAngularSpeed)
	{
		m_angularVelocity.Normalize();
		m_angularVelocity *= maxAngularSpeed;
	}
}

Mat3 Body::GetInverseInertiaTensorLocalSpace() const
{
	Mat3 inertiaTensor = m_shape->InertiaTensor();
	Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;
	return invInertiaTensor;
}

Mat3 Body::GetInverseInertiaTensorWorldSpace() const
{
	Mat3 inertiaTensor = m_shape->InertiaTensor();
	Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;

	Mat3 orient = m_orientation.ToMat3();
	invInertiaTensor = orient * invInertiaTensor * orient.Transpose();
	return invInertiaTensor;
}