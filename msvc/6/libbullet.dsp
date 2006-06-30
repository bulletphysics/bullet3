# Microsoft Developer Studio Project File - Name="libbullet" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=libbullet - Win32 Release
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "libbullet.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "libbullet.mak" CFG="libbullet - Win32 Release"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "libbullet - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "libbullet - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "libbullet - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "release"
# PROP BASE Intermediate_Dir "release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\out\release6\build\libbullet\"
# PROP Intermediate_Dir "..\..\out\release6\build\libbullet\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /Gy /GF /MD /Ob2 /Og /Oi /Ot /Oy /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\Bullet" /I "..\..\BulletDynamics" /I "..\..\LinearMath"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\libbullet\libbullet.res" /i "." /i "..\.." /i "..\..\Bullet" /i "..\..\BulletDynamics" /i "..\..\LinearMath"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\release6\libs\libbullet.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /OPT:NOREF /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ELSEIF  "$(CFG)" == "libbullet - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "debug"
# PROP BASE Intermediate_Dir "debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\out\debug6\build\libbullet\"
# PROP Intermediate_Dir "..\..\out\debug6\build\libbullet\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /GR /MDd /ZI /Od /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\Bullet" /I "..\..\BulletDynamics" /I "..\..\LinearMath"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\libbullet\libbullet.res" /i "." /i "..\.." /i "..\..\Bullet" /i "..\..\BulletDynamics" /i "..\..\LinearMath"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\debug6\libs\libbullet_d.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "libbullet - Win32 Release"
# Name "libbullet - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Bullet\BroadphaseCollision\AxisSweep3.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\BroadphaseCollision\BroadphaseProxy.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\BroadphaseCollision\CollisionAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\BroadphaseCollision\Dispatcher.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\BroadphaseCollision\OverlappingPairCache.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\BroadphaseCollision\SimpleBroadphase.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\CollisionDispatcher.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\CollisionObject.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\CollisionWorld.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\CompoundCollisionAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\ConvexConcaveCollisionAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\ConvexConvexAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\EmptyCollisionAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\ManifoldResult.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\UnionFind.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\BoxShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\BvhTriangleMeshShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\CollisionShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\CompoundShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\ConeShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\ConvexHullShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\ConvexShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\ConvexTriangleMeshShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\CylinderShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\EmptyShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\MinkowskiSumShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\MultiSphereShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\OptimizedBvh.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\PolyhedralConvexShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\Simplex1to4Shape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\SphereShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\StridingMeshInterface.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\TriangleCallback.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\TriangleIndexVertexArray.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\TriangleMesh.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\TriangleMeshShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\ContinuousConvexCollision.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\ConvexCast.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\GjkConvexCast.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\GjkPairDetector.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\Hull.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\ManifoldContactAddResult.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\MinkowskiPenetrationDepthSolver.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\PersistentManifold.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\RaycastCallback.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\Shape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\SubSimplexConvexCast.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\VoronoiSimplexSolver.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Bullet\BroadphaseCollision\AxisSweep3.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\BroadphaseCollision\BroadphaseInterface.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\BroadphaseCollision\BroadphaseProxy.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\BroadphaseCollision\CollisionAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\BroadphaseCollision\Dispatcher.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\BroadphaseCollision\OverlappingPairCache.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\BroadphaseCollision\SimpleBroadphase.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\CollisionCreateFunc.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\CollisionDispatcher.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\CollisionObject.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\CollisionWorld.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\CompoundCollisionAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\ConvexConcaveCollisionAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\ConvexConvexAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\EmptyCollisionAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\ManifoldResult.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionDispatch\UnionFind.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\BoxShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\BvhTriangleMeshShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\CollisionMargin.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\CollisionShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\CompoundShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\ConeShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\ConvexHullShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\ConvexShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\ConvexTriangleMeshShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\CylinderShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\EmptyShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\MinkowskiSumShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\MultiSphereShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\OptimizedBvh.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\PolyhedralConvexShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\Simplex1to4Shape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\SphereShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\StridingMeshInterface.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\TriangleCallback.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\TriangleIndexVertexArray.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\TriangleMesh.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\TriangleMeshShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\CollisionShapes\TriangleShape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\CollisionMargin.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\ContinuousConvexCollision.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\ConvexCast.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\ConvexPenetrationDepthSolver.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\DiscreteCollisionDetectorInterface.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\GjkConvexCast.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\GjkPairDetector.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\Hull.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\HullContactCollector.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\ManifoldContactAddResult.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\ManifoldPoint.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\MinkowskiPenetrationDepthSolver.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\PersistentManifold.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\PointCollector.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\RaycastCallback.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\Shape.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\SimplexSolverInterface.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\SubSimplexConvexCast.h
# End Source File
# Begin Source File

SOURCE=..\..\Bullet\NarrowPhaseCollision\VoronoiSimplexSolver.h
# End Source File
# End Group
# End Target
# End Project
