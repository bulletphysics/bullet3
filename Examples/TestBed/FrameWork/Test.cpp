/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "Test.h"
#include "Render.h"

//#include "freeglut/gl/glut.h"
//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <stdio.h>


Test::Test()
{
	m_textLine = 30;
	
}

Test::~Test()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
}
/*
void Test::MouseDown(const b2Vec2& p)
{
	b2Assert(m_mouseJoint == NULL);

	// Make a small box.
	b2AABB aabb;
	b2Vec2 d;
	d.Set(0.001f, 0.001f);
	aabb.minVertex = p - d;
	aabb.maxVertex = p + d;

	// Query the world for overlapping shapes.
	const int32 k_maxCount = 10;
	b2Shape* shapes[k_maxCount];
	int32 count = m_world->Query(aabb, shapes, k_maxCount);
	b2Body* body = NULL;
	for (int32 i = 0; i < count; ++i)
	{
		if (shapes[i]->m_body->IsStatic() == false)
		{
			bool inside = shapes[i]->TestPoint(p);
			if (inside)
			{
				body = shapes[i]->m_body;
				break;
			}
		}
	}

	if (body)
	{
		b2MouseJointDef md;
		md.body1 = m_world->m_groundBody;
		md.body2 = body;
		md.target = p;
		md.motorForce = 400.0f * body->m_mass;
		m_mouseJoint = (b2MouseJoint*)m_world->CreateJoint(&md);
		body->WakeUp();
	}
}
*/

void Test::MouseUp()
{
/*
	if (m_mouseJoint)
	{
		m_world->DestroyJoint(m_mouseJoint);
		m_mouseJoint = NULL;
	}
*/
}

/*void Test::MouseMove(const b2Vec2& p)
{

	if (m_mouseJoint)
	{
		m_mouseJoint->SetTarget(p);
	}

}
*/

void Test::LaunchBomb()
{
/*
	if (m_bomb)
	{
		m_world->DestroyBody(m_bomb);
		m_bomb = NULL;
	}

	b2BoxDef sd;
	float32 a = 0.5f;
	sd.type = e_boxShape;
	sd.extents.Set(a, a);
	sd.density = 20.0f;

	b2BodyDef bd;
	bd.AddShape(&sd);
	bd.allowSleep = true;
	bd.position.Set(b2Random(-15.0f, 15.0f), 30.0f);
	bd.rotation = b2Random(-1.5f, 1.5f);
	bd.linearVelocity = -1.0f * bd.position;
	bd.angularVelocity = b2Random(-20.0f, 20.0f);

	m_bomb = m_world->CreateBody(&bd);
*/
}

//typedef const char *(APIENTRY * WGLGETEXTENSIONSSTRINGEXT_T)( void );

void Test::Step(const Settings* settings)
{
/*
	float32 timeStep = settings->hz > 0.0f ? 1.0f / settings->hz : 0.0f;

	if (settings->pause)
	{
		timeStep = 0.0f;
	}

	b2World::s_enableWarmStarting = settings->enableWarmStarting;
	b2World::s_enablePositionCorrection = settings->enablePositionCorrection;

	m_world->Step(timeStep, settings->iterationCount);

	m_world->m_broadPhase->Validate();

	for (b2Body* b = m_world->m_bodyList; b; b = b->m_next)
	{
		for (b2Shape* s = b->m_shapeList; s; s = s->m_next)
		{
			if (b->IsStatic())
			{
				DrawShape(s, Color(0.5f, 0.9f, 0.5f));
			}
			else if (b->IsSleeping())
			{
				DrawShape(s, Color(0.5f, 0.5f, 0.9f));
			}
			else if (b == m_bomb)
			{
				DrawShape(s, Color(0.9f, 0.9f, 0.4f));
			}
			else
			{
				DrawShape(s, Color(0.9f, 0.9f, 0.9f));
			}
		}
	}

	for (b2Joint* j = m_world->m_jointList; j; j = j->m_next)
	{
		if (j != m_mouseJoint)
		{
			DrawJoint(j);
		}
	}

	if (settings->drawContacts)
	{
		for (b2Contact* c = m_world->m_contactList; c; c = c->m_next)
		{
			b2Manifold* ms = c->GetManifolds();
			for (int32 i = 0; i < c->GetManifoldCount(); ++i)
			{
				b2Manifold* m = ms + i;
				glPointSize(4.0f);
				glColor3f(1.0f, 0.0f, 0.0f);
				glBegin(GL_POINTS);
				for (int j = 0; j < m->pointCount; ++j)
				{
					b2Vec2 v = m->points[j].position;
					glVertex2f(v.x, v.y);
				}
				glEnd();
				glPointSize(1.0f);
			}
		}
	}

	if (settings->drawImpulses)
	{
		for (b2Contact* c = m_world->m_contactList; c; c = c->m_next)
		{
			b2Manifold* ms = c->GetManifolds();
			for (int32 i = 0; i < c->GetManifoldCount(); ++i)
			{
				b2Manifold* m = ms + i;

				glColor3f(0.9f, 0.9f, 0.3f);
				glBegin(GL_LINES);
				for (int32 j = 0; j < m->pointCount; ++j)
				{
					b2Vec2 v1 = m->points[j].position;
					b2Vec2 v2 = v1 + m->points[j].normalImpulse * m->normal;
					glVertex2f(v1.x, v1.y);
					glVertex2f(v2.x, v2.y);
				}
				glEnd();
			}
		}
	}

	if (settings->drawPairs)
	{
		b2BroadPhase* bp = m_world->m_broadPhase;
		b2Vec2 invQ;
		invQ.Set(1.0f / bp->m_quantizationFactor.x, 1.0f / bp->m_quantizationFactor.y);
		glColor3f(0.9f, 0.9f, 0.3f);
		glBegin(GL_LINES);
		for (int32 i = 0; i < bp->m_pairManager.m_pairCount; ++i)
		{
			b2Pair* pair = bp->m_pairManager.m_pairs + i;
			uint16 id1 = pair->proxyId1;
			uint16 id2 = pair->proxyId2;
			b2Proxy* p1 = bp->m_proxyPool + id1;
			b2Proxy* p2 = bp->m_proxyPool + id2;

			b2AABB b1, b2;
			b1.minVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p1->lowerBounds[0]].value;
			b1.minVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p1->lowerBounds[1]].value;
			b1.maxVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p1->upperBounds[0]].value;
			b1.maxVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p1->upperBounds[1]].value;
			b2.minVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p2->lowerBounds[0]].value;
			b2.minVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p2->lowerBounds[1]].value;
			b2.maxVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p2->upperBounds[0]].value;
			b2.maxVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p2->upperBounds[1]].value;

			b2Vec2 x1 = 0.5f * (b1.minVertex + b1.maxVertex);
			b2Vec2 x2 = 0.5f * (b2.minVertex + b2.maxVertex);

			glVertex2f(x1.x, x1.y);
			glVertex2f(x2.x, x2.y);
		}
		glEnd();
	}

	if (settings->drawAABBs)
	{
		b2BroadPhase* bp = m_world->m_broadPhase;
		b2Vec2 invQ;
		invQ.Set(1.0f / bp->m_quantizationFactor.x, 1.0f / bp->m_quantizationFactor.y);
		glColor3f(0.9f, 0.3f, 0.9f);
		for (int32 i = 0; i < b2_maxProxies; ++i)
		{
			b2Proxy* p = bp->m_proxyPool + i;
			if (p->IsValid() == false)
			{
				continue;
			}

			b2AABB b;
			b.minVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p->lowerBounds[0]].value;
			b.minVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p->lowerBounds[1]].value;
			b.maxVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p->upperBounds[0]].value;
			b.maxVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p->upperBounds[1]].value;

			glBegin(GL_LINE_LOOP);
			glVertex2f(b.minVertex.x, b.minVertex.y);
			glVertex2f(b.maxVertex.x, b.minVertex.y);
			glVertex2f(b.maxVertex.x, b.maxVertex.y);
			glVertex2f(b.minVertex.x, b.maxVertex.y);
			glEnd();
		}
	}

	if (settings->drawStats)
	{
		DrawString(5, m_textLine, "proxies(max) = %d(%d), pairs(max) = %d(%d)",
			m_world->m_broadPhase->m_proxyCount, b2_maxProxies,
			m_world->m_broadPhase->m_pairManager.m_pairCount, b2_maxPairs);

		m_textLine += 15;

		DrawString(5, m_textLine, "bodies/contacts/joints = %d/%d/%d",
			m_world->m_bodyCount, m_world->m_contactCount, m_world->m_jointCount);

		m_textLine += 15;
	}

	if (m_mouseJoint)
	{
		b2Body* body = m_mouseJoint->m_body2;
		b2Vec2 p1 = body->m_position + b2Mul(body->m_R, m_mouseJoint->m_localAnchor);
		b2Vec2 p2 = m_mouseJoint->m_target;

		glPointSize(4.0f);
		glColor3f(0.0f, 1.0f, 0.0f);
		glBegin(GL_POINTS);
		glVertex2f(p1.x, p1.y);
		glVertex2f(p2.x, p2.y);
		glEnd();
		glPointSize(1.0f);

		glColor3f(0.8f, 0.8f, 0.8f);
		glBegin(GL_LINES);
		glVertex2f(p1.x, p1.y);
		glVertex2f(p2.x, p2.y);
		glEnd();
	}
*/

}
