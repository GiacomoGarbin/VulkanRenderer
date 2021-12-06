//
//  Intersections.cpp
//
#include "Intersections.h"
#include "GJK.h"

bool RaySphere(const Vec3& rayStart,
			   const Vec3& rayDir,
			   const Vec3& sphereCenter,
			   const float sphereRadius,
			   float& t1,
			   float& t2)
{
	const Vec3 m = sphereCenter - rayStart;
	
	const float a = rayDir.Dot(rayDir);
	const float b = m.Dot(rayDir);
	const float c = m.Dot(m) - sphereRadius * sphereRadius;

	const float delta = b * b - a * c;
	
	if (delta < 0)
	{
		// no real solutions
		return false;
	}

	const float invA = 1.0f / a;
	const float deltaRoot = sqrtf(delta);
	t1 = invA * (b - deltaRoot);
	t2 = invA * (b + deltaRoot);

	return true;
}

bool SphereSphereDynamic(const ShapeSphere* shapeA,
						 const ShapeSphere* shapeB,
						 const Vec3& posA,
						 const Vec3& posB,
						 const Vec3& velA,
						 const Vec3& velB,
						 const float dt,
						 Vec3& ptOnA,
						 Vec3& ptOnB,
						 float& toi)
{
	const Vec3 relativeVelocity = velA - velB;

	const Vec3 startPtA = posA;
	const Vec3 endPtA = posA + relativeVelocity * dt;
	const Vec3 rayDir = endPtA - startPtA;

	float t0 = 0;
	float t1 = 0;

	if (rayDir.GetLengthSqr() < 0.001f * 0.001f)
	{
		// ray is too short, check if already intersecting
		Vec3 ab = posB - posA;
		float radius = shapeA->m_radius + shapeB->m_radius + 0.001f;
		
		if (ab.GetLengthSqr() > radius * radius)
		{
			return false;
		}
	}
	else if (!RaySphere(posA, rayDir, posB, shapeA->m_radius + shapeB->m_radius, t0, t1))
	{
		return false;
	}

	// from [0,1] to [0,dt]
	t0 *= dt;
	t1 *= dt;

	if (t1 < 0.0f)
	{
		// collision is in the past
		return false;
	}

	// get the earliest positive time of impact
	toi = (t0 < 0.0f) ? 0.0f : t0;

	if (toi > dt)
	{
		// the earliest collision is too far in the future
		return false;
	}

	// get the points of collision
	Vec3 newPosA = posA + velA * toi;
	Vec3 newPosB = posB + velB * toi;
	Vec3 ab = newPosB - newPosA;
	ab.Normalize();

	ptOnA = newPosA + ab * shapeA->m_radius;
	ptOnB = newPosB - ab * shapeB->m_radius;
	return true;
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect(Body* bodyA, Body* bodyB)
{
	const Vec3 ab = bodyB->m_position - bodyA->m_position;

	const ShapeSphere* sphereA = (const ShapeSphere*)bodyA->m_shape;
	const ShapeSphere* sphereB = (const ShapeSphere*)bodyB->m_shape;

	const float radiusAB = sphereA->m_radius + sphereB->m_radius;
	const float lengthSquare = ab.GetLengthSqr();

	if (lengthSquare <= (radiusAB * radiusAB))
	{
		return true;
	}

	return false;
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect(Body* bodyA, Body* bodyB, contact_t& contact)
{
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;

	const Vec3 ab = bodyB->m_position - bodyA->m_position;

	contact.normal = ab;
	contact.normal.Normalize();


	const ShapeSphere* sphereA = (const ShapeSphere*)bodyA->m_shape;
	const ShapeSphere* sphereB = (const ShapeSphere*)bodyB->m_shape;

	contact.ptOnA_WorldSpace = bodyA->m_position + contact.normal * sphereA->m_radius;
	contact.ptOnB_WorldSpace = bodyB->m_position - contact.normal * sphereB->m_radius;

	const float radiusAB = sphereA->m_radius + sphereB->m_radius;
	const float lengthSquare = ab.GetLengthSqr();

	if (lengthSquare <= (radiusAB * radiusAB))
	{
		return true;
	}

	return false;
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect(Body* bodyA, Body* bodyB, const float dt, contact_t& contact)
{
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;

	if (bodyA->m_shape->GetType() == Shape::SHAPE_SPHERE && bodyB->m_shape->GetType() == Shape::SHAPE_SPHERE)
	{
		const ShapeSphere* sphereA = (const ShapeSphere*)bodyA->m_shape;
		const ShapeSphere* sphereB = (const ShapeSphere*)bodyB->m_shape;

		Vec3 posA = bodyA->m_position;
		Vec3 posB = bodyB->m_position;

		Vec3 velA = bodyA->m_linearVelocity;
		Vec3 velB = bodyB->m_linearVelocity;

		if (SphereSphereDynamic(sphereA,
								sphereB,
								posA,
								posB,
								velA,
								velB,
								dt,
								contact.ptOnA_WorldSpace,
								contact.ptOnB_WorldSpace,
								contact.timeOfImpact))
		{
			// step bodies forward to get local space collision points
			bodyA->Update(contact.timeOfImpact);
			bodyB->Update(contact.timeOfImpact);

			// world space contact points to local space
			contact.ptOnA_LocalSpace = bodyA->WorldSpaceToLocalSpace(contact.ptOnA_WorldSpace);
			contact.ptOnB_LocalSpace = bodyB->WorldSpaceToLocalSpace(contact.ptOnB_WorldSpace);

			contact.normal = bodyA->m_position - bodyB->m_position;
			contact.normal.Normalize();

			// rewind time steps
			bodyA->Update(-contact.timeOfImpact);
			bodyB->Update(-contact.timeOfImpact);

			// calculate separation distance
			Vec3 ab = bodyB->m_position - bodyA->m_position;
			float r = ab.GetMagnitude() - (sphereA->m_radius + sphereB->m_radius);
			contact.separationDistance = r;

			return true;
		}
	}

	return false;
}






















