//
//	Intersections.h
//
#pragma once
#include "Contact.h"

bool RaySphere(const Vec3& rayStart,
			   const Vec3& rayDir,
			   const Vec3& sphereCenter,
			   const float sphereRadius,
			   float& t1,
			   float& t2);

bool SphereSphereDynamic(const ShapeSphere* shapeA,
						 const ShapeSphere* shapeB,
						 const Vec3& posA,
						 const Vec3& posB,
						 const Vec3& velA, 
						 const Vec3& velB,
						 const float dt,
						 Vec3& ptOnA,
						 Vec3& ptOnB,
						 float& toi);

bool Intersect(Body* bodyA, Body* bodyB);
bool Intersect(Body* bodyA, Body* bodyB, contact_t& contact);
bool Intersect(Body* bodyA, Body* bodyB, const float dt, contact_t& contact);