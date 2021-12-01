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

	bodyA->m_linearVelocity.Zero();
	bodyB->m_linearVelocity.Zero();

	// move colliding objscts just outside each other
	const float tA = bodyA->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);
	const float tB = bodyB->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);

	const Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;

	bodyA->m_position += ds * tA;
	bodyA->m_position -= ds * tB;
}