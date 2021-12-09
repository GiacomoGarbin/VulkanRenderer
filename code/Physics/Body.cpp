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

// impulsePoint : world space location of the application of the impulse
// impulse      : world space magnitude and direction of the impulse
void Body::ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse)
{
	if (m_invMass == 0.0f)
	{
		return;
	}

	ApplyImpulseLinear(impulse);

	// torque through the center of mass
	Vec3 position = GetCenterOfMassWorldSpace();
	Vec3 r = impulsePoint - position;
	Vec3 dL = r.Cross(impulse);
	ApplyImpulseAngular(dL);
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

Vec3 Body::GetCenterOfMassLocalSpace() const
{
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	return centerOfMass;
}

Vec3 Body::GetCenterOfMassWorldSpace() const
{
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	const Vec3 position = m_position + m_orientation.RotatePoint(centerOfMass);
	return position;
}

void Body::Update(const float dt_sec)
{
	// update position (linear velocity)
	m_position += m_linearVelocity * dt_sec;

	Vec3 centerOfMass = GetCenterOfMassWorldSpace();
	Vec3 centerOfMassToPosition = m_position - centerOfMass;

	// total torque = external torques + internal torque (precession)
	Mat3 orientation = m_orientation.ToMat3();
	Mat3 inertiaTensor = orientation * m_shape->InertiaTensor() * orientation.Transpose();
	Vec3 alpha = inertiaTensor.Inverse() * (m_angularVelocity.Cross(inertiaTensor * m_angularVelocity));
	m_angularVelocity += alpha * dt_sec;

	// update orientation
	Vec3 dAngle = m_angularVelocity * dt_sec;
	Quat dq = Quat(dAngle, dAngle.GetMagnitude());
	m_orientation = dq * m_orientation;
	m_orientation.Normalize();

	// update position (total torque)
	m_position = centerOfMass + dq.RotatePoint(centerOfMassToPosition);
}

Vec3 Body::WorldSpaceToLocalSpace(const Vec3& worldPt) const
{
	Vec3 tmp = worldPt - GetCenterOfMassWorldSpace();
	Quat inverseOrient = m_orientation.Inverse();
	Vec3 localPt = inverseOrient.RotatePoint(tmp);
	return localPt;
}

Vec3 Body::LocalSpaceToWorldSpace(const Vec3& localPt) const
{
	Vec3 worldPt = GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(localPt);
	return worldPt;
}