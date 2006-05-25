/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/*
 * Export a DIF (Dynamics Interchange Format) file.
 */


// @@@ TODO:
//	* export all spaces, and geoms in spaces, not just ones attached to bodies
//	  (separate export function?)
//	* say the space each geom is in, so reader can construct space heirarchy
//	* limot --> separate out into limits and motors?
//	* make sure ODE-specific parameters divided out


#include "ode/ode.h"
#include "objects.h"
#include "joint.h"
#include "collision_kernel.h"

//***************************************************************************
// utility

struct PrintingContext {
	FILE *file;		// file to write to
	int precision;		// digits of precision to print
	int indent;		// number of levels of indent

	void printIndent();
	void printReal (dReal x);
	void print (const char *name, int x);
	void print (const char *name, dReal x);
	void print (const char *name, const dReal *x, int n=3);
	void print (const char *name, const char *x=0);
	void printNonzero (const char *name, dReal x);
	void printNonzero (const char *name, const dReal x[3]);
};


void PrintingContext::printIndent()
{
	for (int i=0; i<indent; i++) fputc ('\t',file);
}


void PrintingContext::print (const char *name, int x)
{
	printIndent();
	fprintf (file,"%s = %d,\n",name,x);
}


void PrintingContext::printReal (dReal x)
{
	if (x==dInfinity) {
		fprintf (file,"inf");
	}
	else if (x==-dInfinity) {
		fprintf (file,"-inf");
	}
	else {
		fprintf (file,"%.*g",precision,x);
	}
}


void PrintingContext::print (const char *name, dReal x)
{
	printIndent();
	fprintf (file,"%s = ",name);
	printReal (x);
	fprintf (file,",\n");
}


void PrintingContext::print (const char *name, const dReal *x, int n)
{
	printIndent();
	fprintf (file,"%s = {",name);
	for (int i=0; i<n; i++) {
		printReal (x[i]);
		if (i < n-1) fputc (',',file);
	}
	fprintf (file,"},\n");
}


void PrintingContext::print (const char *name, const char *x)
{
	printIndent();
	if (x) {
		fprintf (file,"%s = \"%s\",\n",name,x);
	}
	else {
		fprintf (file,"%s\n",name);
	}
}


void PrintingContext::printNonzero (const char *name, dReal x)
{
	if (x != 0) print (name,x);
}


void PrintingContext::printNonzero (const char *name, const dReal x[3])
{
	if (x[0] != 0 && x[1] != 0 && x[2] != 0) print (name,x);
}

//***************************************************************************
// joints


static void printLimot (PrintingContext &c, dxJointLimitMotor &limot, int num)
{
	if (num >= 0) {
		c.printIndent();
		fprintf (c.file,"limit%d = {\n",num);
	}
	else {
		c.print ("limit = {");
	}
	c.indent++;
	c.print ("low_stop",limot.lostop);
	c.print ("high_stop",limot.histop);
	c.printNonzero ("bounce",limot.bounce);
	c.print ("ODE = {");
	c.indent++;
	c.printNonzero ("stop_erp",limot.stop_erp);
	c.printNonzero ("stop_cfm",limot.stop_cfm);
	c.indent--;
	c.print ("},");
	c.indent--;
	c.print ("},");

	if (num >= 0) {
		c.printIndent();
		fprintf (c.file,"motor%d = {\n",num);
	}
	else {
		c.print ("motor = {");
	}
	c.indent++;
	c.printNonzero ("vel",limot.vel);
	c.printNonzero ("fmax",limot.fmax);
	c.print ("ODE = {");
	c.indent++;
	c.printNonzero ("fudge_factor",limot.fudge_factor);
	c.printNonzero ("normal_cfm",limot.normal_cfm);
	c.indent--;
	c.print ("},");
	c.indent--;
	c.print ("},");
}


static const char *getJointName (dxJoint *j)
{
	switch (j->vtable->typenum) {
		case dJointTypeBall: return "ball";
		case dJointTypeHinge: return "hinge";
		case dJointTypeSlider: return "slider";
		case dJointTypeContact: return "contact";
		case dJointTypeUniversal: return "universal";
		case dJointTypeHinge2: return "ODE_hinge2";
		case dJointTypeFixed: return "fixed";
		case dJointTypeNull: return "null";
		case dJointTypeAMotor: return "ODE_angular_motor";
	}
	return "unknown";
}


static void printBall (PrintingContext &c, dxJoint *j)
{
	dxJointBall *b = (dxJointBall*) j;
	c.print ("anchor1",b->anchor1);
	c.print ("anchor2",b->anchor2);
}


static void printHinge (PrintingContext &c, dxJoint *j)
{
	dxJointHinge *h = (dxJointHinge*) j;
	c.print ("anchor1",h->anchor1);
	c.print ("anchor2",h->anchor2);
	c.print ("axis1",h->axis1);
	c.print ("axis2",h->axis2);
	c.print ("qrel",h->qrel,4);
	printLimot (c,h->limot,-1);
}


static void printSlider (PrintingContext &c, dxJoint *j)
{
	dxJointSlider *s = (dxJointSlider*) j;
	c.print ("axis1",s->axis1);
	c.print ("qrel",s->qrel,4);
	c.print ("offset",s->offset);
	printLimot (c,s->limot,-1);
}


static void printContact (PrintingContext &c, dxJoint *j)
{
	dxJointContact *ct = (dxJointContact*) j;
	int mode = ct->contact.surface.mode;
	c.print ("pos",ct->contact.geom.pos);
	c.print ("normal",ct->contact.geom.normal);
	c.print ("depth",ct->contact.geom.depth);
	//@@@ may want to write the geoms g1 and g2 that are involved, for debugging.
	//    to do this we must have written out all geoms in all spaces, not just
	//    geoms that are attached to bodies.
	c.print ("mu",ct->contact.surface.mu);
	if (mode & dContactMu2) c.print ("mu2",ct->contact.surface.mu2);
	if (mode & dContactBounce) c.print ("bounce",ct->contact.surface.bounce);
	if (mode & dContactBounce) c.print ("bounce_vel",ct->contact.surface.bounce_vel);
	if (mode & dContactSoftERP) c.print ("soft_ERP",ct->contact.surface.soft_erp);
	if (mode & dContactSoftCFM) c.print ("soft_CFM",ct->contact.surface.soft_cfm);
	if (mode & dContactMotion1) c.print ("motion1",ct->contact.surface.motion1);
	if (mode & dContactMotion2) c.print ("motion2",ct->contact.surface.motion2);
	if (mode & dContactSlip1) c.print ("slip1",ct->contact.surface.slip1);
	if (mode & dContactSlip2) c.print ("slip2",ct->contact.surface.slip2);
	int fa = 0;		// friction approximation code
	if (mode & dContactApprox1_1) fa |= 1;
	if (mode & dContactApprox1_2) fa |= 2;
	if (fa) c.print ("friction_approximation",fa);
	if (mode & dContactFDir1) c.print ("fdir1",ct->contact.fdir1);
}


static void printUniversal (PrintingContext &c, dxJoint *j)
{
	dxJointUniversal *u = (dxJointUniversal*) j;
	c.print ("anchor1",u->anchor1);
	c.print ("anchor2",u->anchor2);
	c.print ("axis1",u->axis1);
	c.print ("axis2",u->axis2);
	c.print ("qrel1",u->qrel1,4);
	c.print ("qrel2",u->qrel2,4);
	printLimot (c,u->limot1,1);
	printLimot (c,u->limot2,2);
}


static void printHinge2 (PrintingContext &c, dxJoint *j)
{
	dxJointHinge2 *h = (dxJointHinge2*) j;
	c.print ("anchor1",h->anchor1);
	c.print ("anchor2",h->anchor2);
	c.print ("axis1",h->axis1);
	c.print ("axis2",h->axis2);
	c.print ("v1",h->v1);	//@@@ much better to write out 'qrel' here, if it's available
	c.print ("v2",h->v2);
	c.print ("susp_erp",h->susp_erp);
	c.print ("susp_cfm",h->susp_cfm);
	printLimot (c,h->limot1,1);
	printLimot (c,h->limot2,2);
}


static void printFixed (PrintingContext &c, dxJoint *j)
{
	dxJointFixed *f = (dxJointFixed*) j;
	c.print ("qrel",f->qrel);
	c.print ("offset",f->offset);
}


static void printAMotor (PrintingContext &c, dxJoint *j)
{
	dxJointAMotor *a = (dxJointAMotor*) j;
	c.print ("num",a->num);
	c.print ("mode",a->mode);
	c.printIndent();
	fprintf (c.file,"rel = {%d,%d,%d},\n",a->rel[0],a->rel[1],a->rel[2]);
	c.print ("axis1",a->axis[0]);
	c.print ("axis2",a->axis[1]);
	c.print ("axis3",a->axis[2]);
	for (int i=0; i<3; i++) printLimot (c,a->limot[i],i+1);
	c.print ("angle1",a->angle[0]);
	c.print ("angle2",a->angle[1]);
	c.print ("angle3",a->angle[2]);
}

//***************************************************************************
// geometry

static void printGeom (PrintingContext &c, dxGeom *g);

static void printSphere (PrintingContext &c, dxGeom *g)
{
	c.print ("type","sphere");
	c.print ("radius",dGeomSphereGetRadius (g));
}


static void printBox (PrintingContext &c, dxGeom *g)
{
	dVector3 sides;
	dGeomBoxGetLengths (g,sides);
	c.print ("type","box");
	c.print ("sides",sides);
}



static void printCCylinder (PrintingContext &c, dxGeom *g)
{
	dReal radius,length;
	dGeomCCylinderGetParams (g,&radius,&length);
	c.print ("type","capsule");
	c.print ("radius",radius);
	c.print ("length",length);
}


static void printPlane (PrintingContext &c, dxGeom *g)
{
	dVector4 e;
	dGeomPlaneGetParams (g,e);
	c.print ("type","plane");
	c.print ("normal",e);
	c.print ("d",e[3]);
}



static void printRay (PrintingContext &c, dxGeom *g)
{
	dReal length = dGeomRayGetLength (g);
	c.print ("type","ray");
	c.print ("length",length);
}



static void printGeomTransform (PrintingContext &c, dxGeom *g)
{
	dxGeom *g2 = dGeomTransformGetGeom (g);
	const dReal *pos = dGeomGetPosition (g2);
	dQuaternion q;
	dGeomGetQuaternion (g2,q);
	c.print ("type","transform");
	c.print ("pos",pos);
	c.print ("q",q,4);
	c.print ("geometry = {");
	c.indent++;
	printGeom (c,g2);
	c.indent--;
	c.print ("}");
}



static void printTriMesh (PrintingContext &c, dxGeom *g)
{
	c.print ("type","trimesh");
	//@@@ i don't think that the trimesh accessor functions are really
	//    sufficient to read out all the triangle data, and anyway we
	//    should have a method of not duplicating trimesh data that is
	//    shared.
}


static void printGeom (PrintingContext &c, dxGeom *g)
{
	unsigned long category = dGeomGetCategoryBits (g);
	if (category != (unsigned long)(~0)) {
		c.printIndent();
		fprintf (c.file,"category_bits = %lu\n",category);
	}
	unsigned long collide = dGeomGetCollideBits (g);
	if (collide != (unsigned long)(~0)) {
		c.printIndent();
		fprintf (c.file,"collide_bits = %lu\n",collide);
	}
	if (!dGeomIsEnabled (g)) {
		c.print ("disabled",1);
	}
	switch (g->type) {
		case dSphereClass: printSphere (c,g); break;
		case dBoxClass: printBox (c,g); break;
		case dCCylinderClass: printCCylinder (c,g); break;
		case dPlaneClass: printPlane (c,g); break;
		case dRayClass: printRay (c,g); break;
		case dGeomTransformClass: printGeomTransform (c,g); break;
		case dTriMeshClass: printTriMesh (c,g); break;
	}
}

//***************************************************************************
// world

void dWorldExportDIF (dWorldID w, FILE *file, const char *prefix)
{
	PrintingContext c;
	c.file = file;
#if defined(dSINGLE)
	c.precision = 7;
#else
	c.precision = 15;
#endif
	c.indent = 1;

	fprintf (file,"-- Dynamics Interchange Format v0.1\n\n%sworld = dynamics.world {\n",prefix);
	c.print ("gravity",w->gravity);
	c.print ("ODE = {");
	c.indent++;
	c.print ("ERP",w->global_erp);
	c.print ("CFM",w->global_cfm);
	c.print ("auto_disable = {");
	c.indent++;
	c.print ("linear_threshold",w->adis.linear_threshold);
	c.print ("angular_threshold",w->adis.angular_threshold);
	c.print ("idle_time",w->adis.idle_time);
	c.print ("idle_steps",w->adis.idle_steps);
	fprintf (file,"\t\t},\n\t},\n}\n");
	c.indent -= 3;

	// bodies
	int num = 0;
	fprintf (file,"%sbody = {}\n",prefix);
	for (dxBody *b=w->firstbody; b; b=(dxBody*)b->next) {
		b->tag = num;
		fprintf (file,"%sbody[%d] = dynamics.body {\n\tworld = %sworld,\n",prefix,num,prefix);
		c.indent++;
		c.print ("pos",b->pos);
		c.print ("q",b->q,4);
		c.print ("lvel",b->lvel);
		c.print ("avel",b->avel);
		c.print ("mass",b->mass.mass);
		fprintf (file,"\tI = {{");
		for (int i=0; i<3; i++) {
			for (int j=0; j<3; j++) {
				c.printReal (b->mass.I[i*4+j]);
				if (j < 2) fputc (',',file);
			}
			if (i < 2) fprintf (file,"},{");
		}
		fprintf (file,"}},\n");
		c.printNonzero ("com",b->mass.c);
		c.print ("ODE = {");
		c.indent++;
		if (b->flags & dxBodyFlagFiniteRotation) c.print ("finite_rotation",1);
		if (b->flags & dxBodyDisabled) c.print ("disabled",1);
		if (b->flags & dxBodyNoGravity) c.print ("no_gravity",1);
		if (b->flags & dxBodyAutoDisable) {
			c.print ("auto_disable = {");
			c.indent++;
			c.print ("linear_threshold",b->adis.linear_threshold);
			c.print ("angular_threshold",b->adis.angular_threshold);
			c.print ("idle_time",b->adis.idle_time);
			c.print ("idle_steps",b->adis.idle_steps);
			c.print ("time_left",b->adis_timeleft);
			c.print ("steps_left",b->adis_stepsleft);
			c.indent--;
			c.print ("},");
		}
		c.printNonzero ("facc",b->facc);
		c.printNonzero ("tacc",b->tacc);
		if (b->flags & dxBodyFlagFiniteRotationAxis) {
			c.print ("finite_rotation_axis",b->finite_rot_axis);
		}
		c.indent--;
		c.print ("},");
		if (b->geom) {
			c.print ("geometry = {");
			c.indent++;
			for (dxGeom *g=b->geom; g; g=g->body_next) {
				c.print ("{");
				c.indent++;
				printGeom (c,g);
				c.indent--;
				c.print ("},");
			}
			c.indent--;
			c.print ("},");
		}
		c.indent--;
		c.print ("}");
		num++;
	}

	// joints
	num = 0;
	fprintf (file,"%sjoint = {}\n",prefix);
	for (dxJoint *j=w->firstjoint; j; j=(dxJoint*)j->next) {
		c.indent++;
		const char *name = getJointName (j);
		fprintf (file,
			"%sjoint[%d] = dynamics.%s_joint {\n"
			"\tworld = %sworld,\n"
			"\tbody = {%sbody[%d]"
			,prefix,num,name,prefix,prefix,j->node[0].body->tag);
		if (j->node[1].body) fprintf (file,",%sbody[%d]",prefix,j->node[1].body->tag);
		fprintf (file,"},\n");
		switch (j->vtable->typenum) {
			case dJointTypeBall: printBall (c,j); break;
			case dJointTypeHinge: printHinge (c,j); break;
			case dJointTypeSlider: printSlider (c,j); break;
			case dJointTypeContact: printContact (c,j); break;
			case dJointTypeUniversal: printUniversal (c,j); break;
			case dJointTypeHinge2: printHinge2 (c,j); break;
			case dJointTypeFixed: printFixed (c,j); break;
			case dJointTypeAMotor: printAMotor (c,j); break;
		}		
		c.indent--;
		c.print ("}");
		num++;
	}
}
