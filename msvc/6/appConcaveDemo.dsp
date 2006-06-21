# Microsoft Developer Studio Project File - Name="appConcaveDemo" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=appConcaveDemo - Win32 Release
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "appConcaveDemo.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "appConcaveDemo.mak" CFG="appConcaveDemo - Win32 Release"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "appConcaveDemo - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE "appConcaveDemo - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "appConcaveDemo - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "release"
# PROP BASE Intermediate_Dir "release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\out\release6\build\appConcaveDemo\"
# PROP Intermediate_Dir "..\..\out\release6\build\appConcaveDemo\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /Gy /GF /MD /Ob2 /Og /Oi /Ot /Oy /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_CONSOLE" /D "WIN32"  /I "." /I "..\.." /I "..\..\Bullet" /I "..\..\BulletDynamics" /I "..\..\LinearMath" /I "..\..\Extras\PhysicsInterface\Common" /I "..\..\Extras\ConvexDecomposition" /I "..\..\Extras\COLLADA_DOM\include" /I "..\..\Extras\COLLADA_DOM\include\1.4" /I "..\..\Extras\LibXML\include" /I "..\..\Glut" /I "..\..\Demos\OpenGL" /I "..\..\Extras\PhysicsInterface\CcdPhysics"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_CONSOLE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\appConcaveDemo\appConcaveDemo.res" /i "." /i "..\.." /i "..\..\Bullet" /i "..\..\BulletDynamics" /i "..\..\LinearMath" /i "..\..\Extras\PhysicsInterface\Common" /i "..\..\Extras\ConvexDecomposition" /i "..\..\Extras\COLLADA_DOM\include" /i "..\..\Extras\COLLADA_DOM\include\1.4" /i "..\..\Extras\LibXML\include" /i "..\..\Glut" /i "..\..\Demos\OpenGL" /i "..\..\Extras\PhysicsInterface\CcdPhysics"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386 /OPT:NOREF /out:"..\..\ConcaveDemo.exe" /subsystem:console /libpath:"..\..\Glut" 
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ELSEIF  "$(CFG)" == "appConcaveDemo - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "debug"
# PROP BASE Intermediate_Dir "debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\out\debug6\build\appConcaveDemo\"
# PROP Intermediate_Dir "..\..\out\debug6\build\appConcaveDemo\"
# PROP Ignore_Export_Lib 1
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /GR /MDd /ZI /Od /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_CONSOLE" /D "WIN32"  /I "." /I "..\.." /I "..\..\Bullet" /I "..\..\BulletDynamics" /I "..\..\LinearMath" /I "..\..\Extras\PhysicsInterface\Common" /I "..\..\Extras\ConvexDecomposition" /I "..\..\Extras\COLLADA_DOM\include" /I "..\..\Extras\COLLADA_DOM\include\1.4" /I "..\..\Extras\LibXML\include" /I "..\..\Glut" /I "..\..\Demos\OpenGL" /I "..\..\Extras\PhysicsInterface\CcdPhysics"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_CONSOLE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\appConcaveDemo\appConcaveDemo.res" /i "." /i "..\.." /i "..\..\Bullet" /i "..\..\BulletDynamics" /i "..\..\LinearMath" /i "..\..\Extras\PhysicsInterface\Common" /i "..\..\Extras\ConvexDecomposition" /i "..\..\Extras\COLLADA_DOM\include" /i "..\..\Extras\COLLADA_DOM\include\1.4" /i "..\..\Extras\LibXML\include" /i "..\..\Glut" /i "..\..\Demos\OpenGL" /i "..\..\Extras\PhysicsInterface\CcdPhysics"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /out:"..\..\ConcaveDemo.exe" /subsystem:console /libpath:"..\..\Glut" 
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "appConcaveDemo - Win32 Release"
# Name "appConcaveDemo - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Demos\ConcaveDemo\ConcavePhysicsDemo.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Demos\ConcaveDemo\MyMotionState.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Demos\ConcaveDemo\MyMotionState.h
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\msvc\appConcaveDemo.rc
# End Source File
# End Group
# End Target
# End Project
