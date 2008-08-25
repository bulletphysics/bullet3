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

//dRigidBodyArrayCmd.h

#ifndef DYN_DRIGIDBODYARRAYCMD_H
#define DYN_DRIGIDBODYARRAYCMD_H

#include <maya/MArgDatabase.h>
#include <maya/MDagModifier.h>
#include <maya/MSelectionList.h>

#include <maya/MPxCommand.h>

class dRigidBodyArrayCmd : public MPxCommand
{
public:
    dRigidBodyArrayCmd();
    virtual ~dRigidBodyArrayCmd();

    static void *creator();
    static MSyntax syntax();

    MStatus doIt(const MArgList &i_mArgList);
    MStatus redoIt();
    MStatus undoIt();
    bool isUndoable() const { return true; }
    bool hasSyntax() const { return true; }

    static MString typeName;

protected:
    MArgDatabase *m_argDatabase;
    MDagModifier *m_dagModifier;

    MSelectionList m_undoSelectionList;
};

#endif
