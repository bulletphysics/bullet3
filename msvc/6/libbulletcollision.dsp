# Microsoft Developer Studio Project File - Name="libbulletcollision" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=libbulletcollision - Win32 Release
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "libbulletcollision.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "libbulletcollision.mak" CFG="libbulletcollision - Win32 Release"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "libbulletcollision - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "libbulletcollision - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "libbulletcollision - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "release"
# PROP BASE Intermediate_Dir "release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\out\release6\build\libbulletcollision\"
# PROP Intermediate_Dir "..\..\out\release6\build\libbulletcollision\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /Gy /GF /MD /Ob2 /Og /Oi /Ot /Oy /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\libbulletcollision\libbulletcollision.res" /i "." /i "..\.." /i "..\..\src"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\release6\libs\libbulletcollision.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /OPT:NOREF /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ELSEIF  "$(CFG)" == "libbulletcollision - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "debug"
# PROP BASE Intermediate_Dir "debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\out\debug6\build\libbulletcollision\"
# PROP Intermediate_Dir "..\..\out\debug6\build\libbulletcollision\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /GR /MDd /ZI /Od /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\libbulletcollision\libbulletcollision.res" /i "." /i "..\.." /i "..\..\src"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\debug6\libs\libbulletcollision_d.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "libbulletcollision - Win32 Release"
# Name "libbulletcollision - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\BulletCollision\BroadphaseCollision\btAxisSweep3.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\BroadphaseCollision\btBroadphaseProxy.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\BroadphaseCollision\btCollisionAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\BroadphaseCollision\btDispatcher.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\BroadphaseCollision\btOverlappingPairCache.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\BroadphaseCollision\btSimpleBroadphase.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btCollisionDispatcher.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btCollisionObject.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btCollisionWorld.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btCompoundCollisionAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btConvexConcaveCollisionAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btConvexConvexAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btEmptyCollisionAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btManifoldResult.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btSimulationIslandManager.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btSphereBoxCollisionAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btSphereSphereCollisionAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btUnionFind.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btBoxShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btBvhTriangleMeshShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btCollisionShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btCompoundShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btConcaveShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btConeShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btConvexHullShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btConvexShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btConvexTriangleMeshShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btCylinderShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btEmptyShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btMinkowskiSumShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btMultiSphereShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btOptimizedBvh.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btPolyhedralConvexShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btSphereShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btStaticPlaneShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btStridingMeshInterface.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btTetrahedronShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btTriangleCallback.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btTriangleIndexVertexArray.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btTriangleMesh.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btTriangleMeshShape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btContinuousConvexCollision.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btConvexCast.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btGjkConvexCast.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btGjkPairDetector.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btManifoldContactAddResult.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btMinkowskiPenetrationDepthSolver.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btPersistentManifold.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btRaycastCallback.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btSubSimplexConvexCast.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btVoronoiSimplexSolver.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\BulletCollision\BroadphaseCollision\btAxisSweep3.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\BroadphaseCollision\btBroadphaseInterface.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\BroadphaseCollision\btBroadphaseProxy.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\BroadphaseCollision\btCollisionAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\BroadphaseCollision\btDispatcher.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\BroadphaseCollision\btOverlappingPairCache.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\BroadphaseCollision\btSimpleBroadphase.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btCollisionCreateFunc.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btCollisionDispatcher.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btCollisionObject.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btCollisionWorld.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btCompoundCollisionAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btConvexConcaveCollisionAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btConvexConvexAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btEmptyCollisionAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btManifoldResult.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btSimulationIslandManager.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btSphereBoxCollisionAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btSphereSphereCollisionAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionDispatch\btUnionFind.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btBoxShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btBvhTriangleMeshShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btCollisionMargin.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btCollisionShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btCompoundShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btConcaveShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btConeShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btConvexHullShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btConvexShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btConvexTriangleMeshShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btCylinderShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btEmptyShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btMinkowskiSumShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btMultiSphereShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btOptimizedBvh.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btPolyhedralConvexShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btSphereShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btStaticPlaneShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btStridingMeshInterface.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btTetrahedronShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btTriangleCallback.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btTriangleIndexVertexArray.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btTriangleMesh.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btTriangleMeshShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\CollisionShapes\btTriangleShape.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btContinuousConvexCollision.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btConvexCast.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btConvexPenetrationDepthSolver.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btDiscreteCollisionDetectorInterface.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btGjkConvexCast.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btGjkPairDetector.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btManifoldContactAddResult.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btManifoldPoint.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btMinkowskiPenetrationDepthSolver.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btPersistentManifold.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btPointCollector.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btRaycastCallback.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btSimplexSolverInterface.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btSubSimplexConvexCast.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletCollision\NarrowPhaseCollision\btVoronoiSimplexSolver.h
# End Source File
# End Group
# End Target
# End Project
