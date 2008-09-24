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
*/

//dSolverCmd.cpp

#include <maya/MItDependencyNodes.h>
#include <maya/MSyntax.h>

#include "dSolverNode.h"
#include "dSolverCmd.h"

MString dSolverCmd::typeName("dSolver");

dSolverCmd::dSolverCmd()
  : m_argDatabase(0),
    m_dgModifier(0)
{
}


dSolverCmd::~dSolverCmd()
{
    if (m_argDatabase) {
        delete m_argDatabase;
    }
    
    if (m_dgModifier) {
        delete m_dgModifier;
    }
}


void *
dSolverCmd::creator()
{
  return new dSolverCmd;
}


MSyntax
dSolverCmd::syntax()
{
    MSyntax syntax;
    syntax.enableQuery(false);
    syntax.enableEdit(false);

    return syntax;
}


MStatus
dSolverCmd::doIt(const MArgList &args)
{
    MStatus stat;
    m_argDatabase = new MArgDatabase(syntax(), args, &stat);
    if (stat == MS::kFailure) {
	return stat;
    }
    return redoIt();
}


MStatus
dSolverCmd::undoIt()
{
  if (m_dgModifier) {
      m_dgModifier->undoIt();
      delete m_dgModifier;
      m_dgModifier = 0;
  }

  return MS::kSuccess;
}


MStatus
dSolverCmd::redoIt()
{
    MStatus stat;

    // see if node exists
    MItDependencyNodes depIt(MFn::kPluginDependNode);
    for (; !depIt.isDone(); depIt.next()) {
	MObject obj = depIt.thisNode();
	if (MFnDependencyNode(obj).typeId() == dSolverNode::typeId) {
	    return stat;
	}
    }

    m_dgModifier = new MDGModifier;

    MObject dSolverObj = m_dgModifier->createNode(dSolverNode::typeId);
    m_dgModifier->doIt();

    // connect the time attribute
    MPlug plgInTime(dSolverObj, dSolverNode::ia_time);
    MItDependencyNodes depNodeIt(MFn::kTime);
    MObject timeObj = depNodeIt.thisNode();
    if (timeObj != MObject::kNullObj) {
	MPlug plgOutTime = MFnDependencyNode(timeObj).findPlug("outTime", false);
	m_dgModifier->connect(plgOutTime, plgInTime);
	m_dgModifier->doIt();
    }
    //force update of the solver on creation
    MPlug plgRigidBodies(dSolverObj, dSolverNode::oa_rigidBodies);
    bool update;
    plgRigidBodies.getValue(update);
    
    setResult(MFnDependencyNode(dSolverObj).name());

    return stat;
}
