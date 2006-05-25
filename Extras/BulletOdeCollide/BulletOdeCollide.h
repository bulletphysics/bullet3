#ifndef _BULLET_ODE_COLLIDE_H
#define _BULLET_ODE_COLLIDE_H

#include <ode/common.h>
#include <ode/contact.h>

int		BulletOdeCollide(dGeomID o1,dGeomID o2,dContactGeom *contact,int maxContacts,int skip);
void	RemoveOdeGeomFromCollisionCache(dGeomID geom);
class	CollisionShape* GetCollisionShapeFromConvex(dGeomID convex);
dGeomID dCreateConvex (dSpaceID space, class CollisionShape* shape);
void	dGeomConvexGetLengths(dGeomID convex, dVector3 result);

#endif //_BULLET_ODE_COLLIDE_H