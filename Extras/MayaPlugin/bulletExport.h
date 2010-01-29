/*
Bullet Continuous Collision Detection and Physics Library Maya Plugin

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

Modified by Roman Ponomarev <rponom@gmail.com>
01/27/2010 : Replaced COLLADA export with Bullet binary export
*/


#ifndef BULLET_EXPORT_H
#define BULLET_EXPORT_H

#include <string.h> 
#include <sys/types.h>
#include <maya/MStatus.h>
#include <maya/MPxCommand.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MArgList.h>
#include <maya/MGlobal.h>
#include <maya/MSelectionList.h>
#include <maya/MItSelectionList.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
//#include <maya/MFnPlugin.h>
#include <maya/MFnMesh.h>
#include <maya/MFnSet.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItMeshEdge.h>
#include <maya/MFloatVector.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MFloatArray.h>
#include <maya/MObjectArray.h>
#include <maya/MObject.h>
//#include <maya/MPlug.h>
#include <maya/MPxFileTranslator.h>
#include <maya/MFnDagNode.h>
#include <maya/MItDag.h>
#include <maya/MDistance.h>
#include <maya/MIntArray.h>
#include <maya/MIOStream.h>


//////////////////////////////////////////////////////////////
class ObjTranslator : public MPxFileTranslator {
public:
                    ObjTranslator () {};
    virtual         ~ObjTranslator () {};
    static void*    creator();

    MStatus         reader ( const MFileObject& file,
                             const MString& optionsString,
                             FileAccessMode mode);

    MStatus         writer ( const MFileObject& file,
                             const MString& optionsString,
                             FileAccessMode mode );
    bool            haveReadMethod () const;
    bool            haveWriteMethod () const;
    MString         defaultExtension () const;
    MFileKind       identifyFile ( const MFileObject& fileName,
                                   const char* buffer,
                                   short size) const;
protected:
	static MString	fExtension;

private:
    MStatus         exportSelected();
    MStatus         exportAll();

private:
	
};


#endif //BULLET_EXPORT_H

