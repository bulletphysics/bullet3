# Microsoft Developer Studio Project File - Name="appGimpactTestDemo" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=appGimpactTestDemo - Win32 Release
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "appGimpactTestDemo.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "appGimpactTestDemo.mak" CFG="appGimpactTestDemo - Win32 Release"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "appGimpactTestDemo - Win32 DebugDoublePrecision" (based on "Win32 (x86) Console Application")
!MESSAGE "appGimpactTestDemo - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE "appGimpactTestDemo - Win32 ReleaseDoublePrecision" (based on "Win32 (x86) Console Application")
!MESSAGE "appGimpactTestDemo - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "appGimpactTestDemo - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "release"
# PROP BASE Intermediate_Dir "release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\out\release6\build\appGimpactTestDemo\"
# PROP Intermediate_Dir "..\..\out\release6\build\appGimpactTestDemo\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /Gy /GF /MD /Ob2 /Og /Oi /Ot /Oy /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_CONSOLE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut" /I "..\..\Demos\OpenGL" /I "..\..\Extras\GIMPACT\include" /I "..\..\Extras\GIMPACTUtils"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_CONSOLE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\appGimpactTestDemo\appGimpactTestDemo.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut" /i "..\..\Demos\OpenGL" /i "..\..\Extras\GIMPACT\include" /i "..\..\Extras\GIMPACTUtils"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386 /OPT:NOREF /out:"..\..\GimpactTestDemo.exe" /subsystem:console /libpath:"..\..\Glut" 

!ELSEIF  "$(CFG)" == "appGimpactTestDemo - Win32 ReleaseDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "release_dbl"
# PROP BASE Intermediate_Dir "release_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\release_dbl6\build\appGimpactTestDemo\"
# PROP Intermediate_Dir "..\..\out\release_dbl6\build\appGimpactTestDemo\"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c  /D "_MT" /D "_MBCS" /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_CONSOLE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut" /I "..\..\Demos\OpenGL" /I "..\..\Extras\GIMPACT\include" /I "..\..\Extras\GIMPACTUtils"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_CONSOLE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release_dbl6\build\appGimpactTestDemo\appGimpactTestDemo.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut" /i "..\..\Demos\OpenGL" /i "..\..\Extras\GIMPACT\include" /i "..\..\Extras\GIMPACTUtils"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386  /out:"..\..\GimpactTestDemo.exe" /subsystem:console /libpath:"..\..\Glut" 

!ELSEIF  "$(CFG)" == "appGimpactTestDemo - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "debug"
# PROP BASE Intermediate_Dir "debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\out\debug6\build\appGimpactTestDemo\"
# PROP Intermediate_Dir "..\..\out\debug6\build\appGimpactTestDemo\"
# PROP Ignore_Export_Lib 1
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /GR /MDd /ZI /Od /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_CONSOLE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut" /I "..\..\Demos\OpenGL" /I "..\..\Extras\GIMPACT\include" /I "..\..\Extras\GIMPACTUtils"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_CONSOLE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\appGimpactTestDemo\appGimpactTestDemo.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut" /i "..\..\Demos\OpenGL" /i "..\..\Extras\GIMPACT\include" /i "..\..\Extras\GIMPACTUtils"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /out:"..\..\GimpactTestDemo.exe" /subsystem:console /libpath:"..\..\Glut" 

!ELSEIF  "$(CFG)" == "appGimpactTestDemo - Win32 DebugDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "debug_dbl"
# PROP BASE Intermediate_Dir "debug_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\debug_dbl6\build\appGimpactTestDemo\"
# PROP Intermediate_Dir "..\..\out\debug_dbl6\build\appGimpactTestDemo\"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c  /D "_MT" /D "_MBCS" /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_CONSOLE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut" /I "..\..\Demos\OpenGL" /I "..\..\Extras\GIMPACT\include" /I "..\..\Extras\GIMPACTUtils"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_CONSOLE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug_dbl6\build\appGimpactTestDemo\appGimpactTestDemo.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut" /i "..\..\Demos\OpenGL" /i "..\..\Extras\GIMPACT\include" /i "..\..\Extras\GIMPACTUtils"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386  /out:"..\..\GimpactTestDemo.exe" /subsystem:console /libpath:"..\..\Glut" 

!ENDIF 

# Begin Target

# Name "appGimpactTestDemo - Win32 Release"
# Name "appGimpactTestDemo - Win32 ReleaseDoublePrecision"
# Name "appGimpactTestDemo - Win32 Debug"
# Name "appGimpactTestDemo - Win32 DebugDoublePrecision"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Demos\GimpactTestDemo\GimpactTestDemo.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Demos\GimpactTestDemo\main.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Demos\GimpactTestDemo\BunnyMesh.h
# End Source File
# Begin Source File

SOURCE=..\..\Demos\GimpactTestDemo\GimpactTestDemo.h
# End Source File
# Begin Source File

SOURCE=..\..\Demos\GimpactTestDemo\TorusMesh.h
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\msvc\appGimpactTestDemo.rc
# End Source File
# End Group
# End Target
# End Project
