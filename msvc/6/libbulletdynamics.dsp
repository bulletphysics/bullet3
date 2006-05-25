# Microsoft Developer Studio Project File - Name="libbulletdynamics" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=libbulletdynamics - Win32 Release
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "libbulletdynamics.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "libbulletdynamics.mak" CFG="libbulletdynamics - Win32 Release"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "libbulletdynamics - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "libbulletdynamics - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "libbulletdynamics - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "release"
# PROP BASE Intermediate_Dir "release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\out\release6\build\libbulletdynamics\"
# PROP Intermediate_Dir "..\..\out\release6\build\libbulletdynamics\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /GX /G5 /FD /c /Gy /GF /MD /Ob2 /Og /Oi /Ot /Oy /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\Bullet" /I "..\..\BulletDynamics" /I "..\..\LinearMath"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\libbulletdynamics\libbulletdynamics.res" /i "." /i "..\.." /i "..\..\Bullet" /i "..\..\BulletDynamics" /i "..\..\LinearMath"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\release6\libs\libbulletdynamics.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /OPT:NOREF /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ELSEIF  "$(CFG)" == "libbulletdynamics - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "debug"
# PROP BASE Intermediate_Dir "debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\out\debug6\build\libbulletdynamics\"
# PROP Intermediate_Dir "..\..\out\debug6\build\libbulletdynamics\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /GX /G5 /FD /c /GR /MDd /ZI /Od /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\Bullet" /I "..\..\BulletDynamics" /I "..\..\LinearMath"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\libbulletdynamics\libbulletdynamics.res" /i "." /i "..\.." /i "..\..\Bullet" /i "..\..\BulletDynamics" /i "..\..\LinearMath"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\debug6\libs\libbulletdynamics_d.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "libbulletdynamics - Win32 Release"
# Name "libbulletdynamics - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\ContactConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\HingeConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\OdeConstraintSolver.cpp
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\Point2PointConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\SimpleConstraintSolver.cpp
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\Solve2LinearConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\SorLcp.cpp
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\TypedConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\Dynamics\BU_Joint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\Dynamics\ContactJoint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\Dynamics\RigidBody.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\ConstraintSolver.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\ContactConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\ContactSolverInfo.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\HingeConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\JacobianEntry.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\OdeConstraintSolver.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\Point2PointConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\SimpleConstraintSolver.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\Solve2LinearConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\SorLcp.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\ConstraintSolver\TypedConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\Dynamics\BU_Joint.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\Dynamics\ContactJoint.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\Dynamics\MassProps.h
# End Source File
# Begin Source File

SOURCE=..\..\BulletDynamics\Dynamics\RigidBody.h
# End Source File
# End Group
# End Target
# End Project
