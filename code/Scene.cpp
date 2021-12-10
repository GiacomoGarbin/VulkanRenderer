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
Scene::~Scene()
{
	for (int i = 0; i < m_bodies.size(); i++)
	{
		delete m_bodies[i].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset()
{
	for (int i = 0; i < m_bodies.size(); i++)
	{
		delete m_bodies[i].m_shape;
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

	for (int x = 0; x < 6; ++x)
	{
		for (int y = 0; y < 6; ++y)
		{
			float radius = 0.5f;
			float xx = float(x - 1) * radius * 1.5f;
			float yy = float(y - 1) * radius * 1.5f;

			body.m_position = Vec3(xx, yy, 10.0f);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity.Zero();
			body.m_invMass = 1.0f;
			body.m_elasticity = 0.5f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);
			m_bodies.push_back(body);
		}
	}

	for (int x = 0; x < 3; ++x)
	{
		for (int y = 0; y < 3; ++y)
		{
			float radius = 80.0f;
			float xx = float(x - 1) * radius * 0.25f;
			float yy = float(y - 1) * radius * 0.25f;

			body.m_position = Vec3(xx, yy, -radius);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity.Zero();
			body.m_invMass = 0.0f;
			body.m_elasticity = 0.99f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);
			m_bodies.push_back(body);
		}
	}
}

int CompareContacts(const void* p1, const void* p2)
{
	contact_t a = *(contact_t*)p1;
	contact_t b = *(contact_t*)p2;

	if (a.timeOfImpact < b.timeOfImpact)
	{
		return -1;
	}

	if (a.timeOfImpact == b.timeOfImpact)
	{
		return 0;
	}

	return +1;
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update(const float dt_sec)
{
	// acceleration due to gravity
	for (int i = 0; i < m_bodies.size(); ++i)
	{
		Body* body = &m_bodies[i];
		float mass = 1.0f / body->m_invMass;

		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
		body->ApplyImpulseLinear(impulseGravity);
	}

	// broadphase
	std::vector<collisionPair_t> collisionPairs;
	BroadPhase(m_bodies.data(), (int)m_bodies.size(), collisionPairs, dt_sec);

	// narrowphase
	int numContacts = 0;
	const int maxContacts = m_bodies.size() * m_bodies.size();
	contact_t* contacts = (contact_t*)alloca(sizeof(contact_t) * maxContacts);

	for (int i = 0; i < collisionPairs.size(); ++i)
	{
		const collisionPair_t& pair = collisionPairs[i];
		Body* bodyA = &m_bodies[pair.a];
		Body* bodyB = &m_bodies[pair.b];

		if (bodyA->m_invMass == 0.0f && bodyB->m_invMass == 0.0f)
		{
			continue;
		}

		contact_t contact;
		if (Intersect(bodyA, bodyB, dt_sec, contact))
		{
			contacts[numContacts] = contact;
			numContacts++;
		}
	}

	// sort times of impact from earliest to latest
	if (numContacts > 1)
	{
		qsort(contacts, numContacts, sizeof(contact_t), CompareContacts);
	}

	float accomulatedTime = 0.0f;

	for (int i = 0; i < numContacts; ++i)
	{
		contact_t& contact = contacts[i];
		const float dt = contact.timeOfImpact - accomulatedTime;

		//Body* bodyA = contact.bodyA;
		//Body* bodyB = contact.bodyB;

		//// skip infinite mass
		//if (bodyA->m_invMass == 0.0f && bodyB->m_invMass == 0.0f)
		//{
		//	continue;
		//}

		// position update
		for (int j = 0; j < m_bodies.size(); ++j)
		{
			m_bodies[j].Update(dt);
		}

		ResolveContact(contact);
		accomulatedTime += dt;
	}

	const float timeRemaning = dt_sec - accomulatedTime;
	if (timeRemaning > 0.0f)
	{
		for (int i = 0; i < m_bodies.size(); ++i)
		{
			m_bodies[i].Update(timeRemaning);
		}
	}
}