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

#include "bulletExport.h"
#include "solver.h"
#include "solver_impl.h"

#if defined (_WIN32)
#define strcasecmp stricmp
#elif defined  (OSMac_)
extern "C" int strcasecmp (const char *, const char *);

#endif

#define NO_SMOOTHING_GROUP      -1
#define INITIALIZE_SMOOTHING    -2
#define INVALID_ID              -1

//////////////////////////////////////////////////////////////

MString ObjTranslator::fExtension = "bullet";

//////////////////////////////////////////////////////////////

void* ObjTranslator::creator()
{
    return new ObjTranslator();
}

//////////////////////////////////////////////////////////////

MStatus ObjTranslator::reader ( const MFileObject& file,
                                const MString& options,
                                FileAccessMode mode)
{
    fprintf(stderr, "Bullet Physics import is not available yet\n");
    return MS::kFailure;
}



MStatus ObjTranslator::writer ( const MFileObject& file,
                                const MString& options,
                                FileAccessMode mode )

{
    MStatus status;
    
    MString mname = file.fullName(), unitName;
   
//just pass in the filename

#if defined (OSMac_)
	char fname[256];//MAXPATHLEN];
	strcpy (fname, file.fullName().asChar());
//	fp = fopen(fname,"wb");//MAYAMACTODO
#else
    const char *fname = mname.asChar();
  //  fp = fopen(fname,"w");
#endif

shared_ptr<solver_impl_t> solv = solver_t::get_solver();

solv->export_bullet_file(fname);

return status;

}
//////////////////////////////////////////////////////////////

bool ObjTranslator::haveReadMethod () const
{
    return true;
}
//////////////////////////////////////////////////////////////

bool ObjTranslator::haveWriteMethod () const
{
    return true;
}
//////////////////////////////////////////////////////////////

MString ObjTranslator::defaultExtension () const
{
//    return MString("bullet");
	return fExtension;
}

MString ObjTranslator::filter() const
{
	//return "*.bullet;*.dae";
	return "*.bullet";
}

//////////////////////////////////////////////////////////////

MPxFileTranslator::MFileKind ObjTranslator::identifyFile (
                                        const MFileObject& fileName,
                                        const char* buffer,
                                        short size) const
{
    const char * name = fileName.name().asChar();
    int   nameLength = strlen(name);
    
    if ((nameLength > 7) && !strcasecmp(name+nameLength-7, ".bullet"))
        return kCouldBeMyFileType;
    else
        return kNotMyFileType;
}
//////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
MStatus ObjTranslator::exportSelected( )
{
	MStatus status;
	MString filename;


	// Create an iterator for the active selection list
	//
	MSelectionList slist;
	MGlobal::getActiveSelectionList( slist );
	MItSelectionList iter( slist );

	if (iter.isDone()) {
	    fprintf(stderr,"Error: Nothing is selected.\n");
	    return MS::kFailure;
	}

	
	return status;
}

//////////////////////////////////////////////////////////////

MStatus ObjTranslator::exportAll( )
{
	MStatus status = MS::kSuccess;

	return status;
}
