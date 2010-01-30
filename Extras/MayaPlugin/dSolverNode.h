/*
Bullet Continuous Collision Detection and Physics Library Maya Plugin
Copyright (c) 2008 Walt Disney Studios
 
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising
from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:
 
1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this
software in a product, an acknowledgment in the product documentation
would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must
not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 
Written by: Nicola Candussi <nicola@fluidinteractive.com>

Modified by Roman Ponomarev <rponom@gmail.com>
01/22/2010 : Constraints reworked
*/

//dSolverNode.h

#ifndef DYN_DSOLVERNODE_H
#define DYN_DSOLVERNODE_H

#include <maya/MString.h>
#include <maya/MTypeId.h>
#include <maya/MPxNode.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MTime.h>

#include <vector>

#include "mathUtils.h"

//class dSolverNode : public MPxNode
class dSolverNode : public MPxLocatorNode
{
public:
    dSolverNode();
    virtual ~dSolverNode();
    virtual void postConstructor(); 
    virtual void	draw(	M3dView & view, const MDagPath & path,
							M3dView::DisplayStyle style,
							M3dView::DisplayStatus status );


	virtual bool            isBounded() const { 
		return false; 
	}
virtual MBoundingBox boundingBox() const
{
    MObject node = thisMObject();
    MPoint corner1(-1, -1, -1);
    MPoint corner2(1, 1, 1);
    return MBoundingBox(corner1, corner2);
}

    virtual bool        excludeAsLocator() const { 
		return false; 
	}
    virtual bool        isTransparent() const { 
		return false; 
	}

    static  void *          creator();
    static  MStatus         initialize();

    virtual bool setInternalValueInContext ( const  MPlug & plug, const  MDataHandle & dataHandle,  MDGContext & ctx );

    virtual MStatus     compute( const MPlug& plug, MDataBlock& data );

    static  MObject     ia_time;
    static  MObject     ia_startTime;
    static  MObject     ia_gravity;
    static  MObject     ia_enabled;
    static  MObject     ia_splitImpulse;
    static  MObject     ia_substeps;
    static  MObject     oa_rigidBodies;

    //Solver Settings
    static  MObject     ssSolverType;
//

	static  MObject     ia_DBG_DrawWireframe;
	static  MObject     ia_DBG_DrawAabb;
	static  MObject     ia_DBG_DrawFeaturesText;
	static  MObject     ia_DBG_DrawContactPoints;
	static  MObject     ia_DBG_NoDeactivation;
	static  MObject     ia_DBG_NoHelpText;
	static  MObject     ia_DBG_DrawText;
	static  MObject     ia_DBG_ProfileTimings;
	static  MObject     ia_DBG_EnableSatComparison;
	static  MObject     ia_DBG_DisableBulletLCP;
	static  MObject     ia_DBG_EnableCCD;
	static  MObject     ia_DBG_DrawConstraints;
	static  MObject     ia_DBG_DrawConstraintLimits;
	static  MObject     ia_DBG_FastWireframe;

    //

public: 
    static  MTypeId	typeId;
    static  MString     typeName;
	static	bool	isStartTime;

	static void updateAllRigidBodies();

protected:
    
    struct xforms_t {
        vec3f m_x0;
        vec3f m_x1;
        quatf m_q0;
        quatf m_q1;
    };

    void computeRigidBodies(const MPlug& plug, MDataBlock& data);
    void dumpRigidBodyArray(MObject &node);
    bool expandFileExpression(std::string const& expr, std::string &base_name, std::string &extension);

    void initRigidBodies(MPlugArray &rbConnections);
    void gatherPassiveTransforms(MPlugArray &rbConnections, std::vector<xforms_t> &xforms);
    void updatePassiveRigidBodies(MPlugArray &rbConnections, std::vector<xforms_t> &xforms, float t);
    void updateActiveRigidBodies(MPlugArray &rbConnections);
    void applyFields(MPlugArray &rbConnections, float dt);
	void updateConstraint(MObject& bodyNode);
protected:
    MTime m_prevTime;
};


#endif
