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