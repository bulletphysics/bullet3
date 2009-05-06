/*
Bullet Continuous Collision Detection and Physics Library Maya Plugin
Copyright (c) 2008 Herbert Law
 
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
 
Written by: Herbert Law <Herbert.Law@gmail.com>
*/

//dSliderConstraintCmd.cpp

#include <maya/MGlobal.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MSyntax.h>

#include "sliderConstraintNode.h"
#include "dSliderConstraintCmd.h"


MString dSliderConstraintCmd::typeName("dSliderConstraint");

dSliderConstraintCmd::dSliderConstraintCmd()
  : m_argDatabase(0),
    m_dagModifier(0)
{
}


dSliderConstraintCmd::~dSliderConstraintCmd()
{
  if (m_argDatabase) {
    delete m_argDatabase;
  }

  if (m_dagModifier) {
    delete m_dagModifier;
  }
}


void *
dSliderConstraintCmd::creator()
{
  return new dSliderConstraintCmd;
}


MSyntax
dSliderConstraintCmd::syntax()
{
    MSyntax syntax;
    syntax.enableQuery(false);
    syntax.enableEdit(false);

    syntax.addFlag("-n", "-name", MSyntax::kString);
  //  syntax.addFlag("-fn", "-filename", MSyntax::kString);
   // syntax.addFlag("-col", "-color", MSyntax::kString);
   // syntax.addFlag("-dia", "-diameter", MSyntax::kDouble);

    return syntax;
}


MStatus
dSliderConstraintCmd::doIt(const MArgList &args)
{
    MStatus stat;
    m_argDatabase = new MArgDatabase(syntax(), args, &stat);
    if (stat == MS::kFailure) {
	return stat;
    }
    return redoIt();
}


MStatus
dSliderConstraintCmd::undoIt()
{
  MGlobal::setActiveSelectionList(m_undoSelectionList);

  if (m_dagModifier) {
      m_dagModifier->undoIt();
      delete m_dagModifier;
      m_dagModifier = 0;
  }

  return MS::kSuccess;
}


MStatus
dSliderConstraintCmd::redoIt()
{
    MGlobal::getActiveSelectionList(m_undoSelectionList);

    MString name;
    if (m_argDatabase->isFlagSet("-name")) {
	m_argDatabase->getFlagArgument("-name", 0, name);
    }
    if (!name.length()) {
	name = "dSliderConstraint";
    }

    m_dagModifier = new MDagModifier;

    MObject parentObj = m_dagModifier->createNode("transform");
    m_dagModifier->renameNode(parentObj, name + "#");
    m_dagModifier->doIt();

    MObject dConstraintObj = m_dagModifier->createNode(sliderConstraintNode::typeId, parentObj);
    std::string dConstraintName = MFnDependencyNode(parentObj).name().asChar();
    std::string::size_type pos = dConstraintName.find_last_not_of("0123456789");
    dConstraintName.insert(pos + 1, "Shape");
    m_dagModifier->renameNode(dConstraintObj, dConstraintName.c_str());
    m_dagModifier->doIt();

    setResult(MFnDependencyNode(dConstraintObj).name());

    return MS::kSuccess;
}
