//
//  Contact.cpp
//
#include "Contact.h"

/*
====================================================
ResolveContact
====================================================
*/
void ResolveContact(contact_t& contact)
{
	Body* bodyA = contact.bodyA;
	Body* bodyB = contact.bodyB;

	const Vec3 ptOnA = contact.ptOnA_WorldSpace;
	const Vec3 ptOnB = contact.ptOnB_WorldSpace;

	const float elasticity = bodyA->m_elasticity * bodyB->m_elasticity;

	const float invMassA = bodyA->m_invMass;
	const float invMassB = bodyB->m_invMass;

	const Mat3 invWorldInertiaA = bodyA->GetInverseInertiaTensorWorldSpace();
	const Mat3 invWorldInertiaB = bodyB->GetInverseInertiaTensorWorldSpace();

	const Vec3& n = contact.normal;

	const Vec3 ra = ptOnA - bodyA->GetCenterOfMassWorldSpace();
	const Vec3 rb = ptOnB - bodyB->GetCenterOfMassWorldSpace();

	const Vec3 angularJA = (invWorldInertiaA * ra.Cross(n)).Cross(ra);
	const Vec3 angularJB = (invWorldInertiaB * rb.Cross(n)).Cross(rb);
	const float angularFactor = (angularJA + angularJB).Dot(n);

	// world space velocity of motion and rotation
	const Vec3 velA = bodyA->m_linearVelocity + bodyA->m_angularVelocity.Cross(ra);
	const Vec3 velB = bodyB->m_linearVelocity + bodyB->m_angularVelocity.Cross(rb);

	// collision impulse
	const Vec3 vab = velA - velB;
	const float J = (1.0f + elasticity) * vab.Dot(n) / (invMassA + invMassB + angularFactor);
	const Vec3 impulse = n * J;

	bodyA->ApplyImpulse(ptOnA, impulse * -1.0f);
	bodyB->ApplyImpulse(ptOnB, impulse * +1.0f);

	// friction impulse
	const float friction = bodyA->m_friction * bodyB->m_friction;

	// normal direction of the velocity with respect to the normal of the collision
	const Vec3 velNorm = n * n.Dot(vab);
	// tangent direction of the velocity with respect to the normal of the collision
	const Vec3 velTang = vab - velNorm;

	Vec3 relativeVelTang = velTang;
	relativeVelTang.Normalize();

	const Vec3 inertiaA = (invWorldInertiaA * ra.Cross(relativeVelTang)).Cross(ra);
	const Vec3 inertiaB = (invWorldInertiaB * rb.Cross(relativeVelTang)).Cross(rb);
	const float invInertia = (inertiaA + inertiaB).Dot(relativeVelTang);

	const float reducedMass = 1.0f / (invMassA + invMassB + invInertia);
	const Vec3 impulseFriction = velTang * reducedMass * friction;

	bodyA->ApplyImpulse(ptOnA, impulseFriction * -1.0f);
	bodyB->ApplyImpulse(ptOnB, impulseFriction * +1.0f);

	// move colliding objects just outside of each other
	const Vec3 ds = ptOnB - ptOnA;
	const float tA = invMassA / (invMassA + invMassB);
	const float tB = invMassB / (invMassA + invMassB);

	bodyA->m_position += ds * tA;
	bodyB->m_position -= ds * tB;
}