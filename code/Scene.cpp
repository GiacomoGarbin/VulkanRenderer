//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize()
{
	Body body;
	body.m_position = Vec3(0, 0, 10);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_invMass = 1.0f;
	body.m_shape = new ShapeSphere(1.0f);
	m_bodies.push_back(body);

	body.m_position = Vec3(0, 0, -1000);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_invMass = 0.0f;
	body.m_shape = new ShapeSphere(1000.0f);
	m_bodies.push_back(body);
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec )
{
	for (int i = 0; i < m_bodies.size(); ++i)
	{
		Body* body = &m_bodies[i];
		float mass = 1.0f / body->m_invMass;

		// acceleration due to gravity
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
		body->ApplyImpulseLinear(impulseGravity);
	}

	for (int i = 0; i < m_bodies.size(); ++i)
	{
		for (int j = i + 1; j < m_bodies.size(); ++j)
		{
			Body* a = &m_bodies[i];
			Body* b = &m_bodies[j];

			if (a->m_invMass == 0.0f && b->m_invMass == 0.0f)
			{
				continue;
			}

			if (Intersect(a, b))
			{
				a->m_linearVelocity.Zero();
				b->m_linearVelocity.Zero();
			}
		}
	}


	for (int i = 0; i < m_bodies.size(); ++i)
	{
		// position update
		m_bodies[i].m_position += m_bodies[i].m_linearVelocity * dt_sec;
	}
}