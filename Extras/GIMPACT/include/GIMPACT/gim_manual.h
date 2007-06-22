#ifndef GIM_MANUAL_H_INCLUDED
#define GIM_MANUAL_H_INCLUDED

/*! \file gim_manual.h
*\author Francisco León Nájera
GIMPACT documentation

*/
/*
-----------------------------------------------------------------------------
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2006 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com

 This library is free software; you can redistribute it and/or
 modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License is included with this library in the
       file GIMPACT-LICENSE-LGPL.TXT.
   (2) The BSD-style license that is included with this library in
       the file GIMPACT-LICENSE-BSD.TXT.
   (3) The zlib/libpng license that is included with this library in
       the file GIMPACT-LICENSE-ZLIB.TXT.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-ZLIB.TXT and GIMPACT-LICENSE-BSD.TXT for more details.

-----------------------------------------------------------------------------
*/

/*! \mainpage GIMPACT
*\section INTRO INTRODUCTION
<p>
<b>GIMPACT</b> is a software library with tools for geometry processing and collision detection, focused on solving most common problems on Virtual Reality development.
</p>
<p>
Since Version 0.2, <b>GIMPACT</b> will work integrated with the <b>Bullet Engine</b> for general collision detection and physics. For  more details, please visit http://www.continuousphysics.com/Bullet/
</p>
<p>For updates, please visit http://sourceforge.net/projects/gimpact/ </p>

*\section CONTENT
- \subpage FEATURES
- \subpage REQUIREMENTS
- \subpage PROGRAMMERS_GUIDE "PROGRAMMERS GUIDE"
- \subpage TODO
*/
/*! \page FEATURES
- C++ oriented.
- Extensible design.
- Collision detection algorithm for concave shapes in the <b>Bullet Physics Engine</b>. See btGImpactCollisionAlgorithm.
- Efficient Dynamic <b>Hierarcht Bounding Box tree structures</b> for handle collisions with composed shapes whose have large sets of primitives (triangles or convex shapes) See the GIM_BOX_TREE_SET class.
- Collision detection for trimeshes with the btGImpactMeshShape class. Now <b>GIMPACT</b> works fine with the btStridingInterface class for getting the data from trimesh models.
- Support for deformable trimeshes.
- Collision detection for compound shapes through the btGImpactCompoundShape class. This class takes advantage from the Hierarcht Bounding Box structure (GIM_BOX_TREE_SET class).
- Collision Shapes (Both btGImpactCompoundSape and btGImpactTrimeshShape ) can be shared by two or more Rigid Bodies in <b>Bullet</b>.

*/
/*! \page REQUIREMENTS
- GIMPACT has dependencies to the <a href = "http://www.continuousphysics.com/Bullet/"> Bullet Physics Engine</a>.
- Ansi C++ compiler,
*/
/*! \page PROGRAMMERS_GUIDE PROGRAMMERS GUIDE
This guide will show how to incorpore the GIMPACt functionality on the Bullet Engine:
*\section REG Registering the Algorithm
For register this algorithm in Bullet, proceed as following:
 \code
btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(m_dynamicsWorld ->getDispatcher());
btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
 \endcode
</p>
With the instructon above, btGImpactCollisionAlgorithm will handle:
- Convex shapes vs GImpact shapes.
- Concave shapes vs GImpact shapes.
- Compoind shapes vs GImpact shapes.

*\section CREATING_SHAPES Creating Shapes.
*\subsection TRIMESH Creating trimeshes.
<p>For creating trimeshes you must provide an interface for your model data. You could use btTriangleIndexVertexArray class for providing the indices and the vertices from your triangle models.</p>
<p>
For example, you could create a trimesh from memory as following:
\code
btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(NUM_TRIANGLES,
		&gIndices[0][0],
		3*sizeof(int),
		NUM_VERTICES,(REAL*) &gVertices[0],sizeof(REAL)*3);
\endcode
</p>
<p>
Where gIndices is an array of integers and gVertices is an array of float with 3 components.
</p>
Then you must create the Trimesh shape as following:
\code
btGImpactMeshShape * trimesh = new btGImpactMeshShape(indexVertexArrays);
\endcode
The next step is configuring the trimesh, for example changing the scale:
\code
trimesh->setLocalScaling(btVector3(4.f,4.f,4.f));
\endcode
At end, you must call btGImpactMeshShape.updateBound for ensure that the shape will build its internal Box set structure:
\code
trimesh->updateBound();// Call this method once before doing collisions
\endcode
Also you must call btGImpactMeshShape.postUpdate() each time when changing the trimesh data ( For deformable meshes), this will enable a flag to the trimesh shape which tells that the trimesh data has been changed and btGImpactMeshShape.updateBound will be called in collision routines.
<

*\subsection COMPOUND Compound Shapes.
For compound shapes, you must create btGImpactCompoundShape objects. Then you could add
sub shapes as following;
\code
btGImpactCompoundShape * mycompound = new btGImpactCompoundShape();

btTransform localtransform;
 .... Setting transformation

//add shapes with transformation
btCollisionShape * subshape = creatingShape(0);
mycompound->addChildShape(localtransform,subshape);
.... Setting transformation
btCollisionShape * subshape2 = creatingShape(1);
mycompound->addChildShape(localtransform,subshape);
.... add more shapes
\endcode
At end, you must call btGImpactCompoundShape.updateBound for ensure that the shape will build its internal Box set structure:
*/

/*! \page TODO
*\section FEATURE FEATURE REQUESTS
- Tetrahedrization for solid trimeshes.
- Examples for implement deformable collisions, like cloth simulations and finite elements dynamics.
- Generic Ray collision functionality.
- Implement Low level SAT algorithms for speed up convex collisions.
*\section MISCELANEOUS
- Improve the documentation.
- Benchmarking.
- Bug reports and Bug Fixes.
*/


#endif
