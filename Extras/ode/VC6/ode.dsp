# Microsoft Developer Studio Project File - Name="ode" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=ode - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "ode.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "ode.mak" CFG="ode - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "ode - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "ode - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""$/TR4/ODE", BHKAAAAA"
# PROP Scc_LocalPath ".."
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "ode - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /MD /W2 /GX /O2 /I "..\Include" /I "..\OPCODE" /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD BASE RSC /l 0x809 /d "NDEBUG"
# ADD RSC /l 0x809 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\lib\ode.lib"

!ELSEIF  "$(CFG)" == "ode - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W2 /Gm /GX /ZI /Od /I "..\Include" /I "..\OPCODE" /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /FR /YX /FD /GZ /c
# ADD BASE RSC /l 0x809 /d "_DEBUG"
# ADD RSC /l 0x809 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\lib\oded.lib"

!ENDIF 

# Begin Target

# Name "ode - Win32 Release"
# Name "ode - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=..\ode\src\array.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_kernel.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_quadtreespace.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_space.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_std.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_transform.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_trimesh.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_trimesh_box.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_trimesh_ccylinder.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_trimesh_distance.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_trimesh_ray.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_trimesh_sphere.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_trimesh_trimesh.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_util.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\error.cpp
# End Source File
# Begin Source File

SOURCE="..\ode\src\export-dif.cpp"
# End Source File
# Begin Source File

SOURCE=..\ode\src\fastdot.c
# End Source File
# Begin Source File

SOURCE=..\ode\src\fastldlt.c
# End Source File
# Begin Source File

SOURCE=..\ode\src\fastlsolve.c
# End Source File
# Begin Source File

SOURCE=..\ode\src\fastltsolve.c
# End Source File
# Begin Source File

SOURCE=..\ode\src\joint.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\lcp.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\mass.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\mat.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\matrix.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\memory.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\misc.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\obstack.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\ode.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\odemath.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\quickstep.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\rotation.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\step.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\stepfast.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\testing.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\timer.cpp
# End Source File
# Begin Source File

SOURCE=..\ode\src\util.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=..\ode\src\array.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_kernel.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_space_internal.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_std.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_transform.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_trimesh_internal.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\collision_util.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\joint.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\lcp.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\mat.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\objects.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\obstack.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\quickstep.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\step.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\testing.h
# End Source File
# Begin Source File

SOURCE=..\ode\src\util.h
# End Source File
# End Group
# End Target
# End Project
