# Microsoft Developer Studio Project File - Name="DrawStuff" - Package Owner=<4>

# Microsoft Developer Studio Generated Build File, Format Version 6.00

# ** DO NOT EDIT **



# TARGTYPE "Win32 (x86) Static Library" 0x0104



CFG=DrawStuff - Win32 Debug

!MESSAGE This is not a valid makefile. To build this project using NMAKE,

!MESSAGE use the Export Makefile command and run

!MESSAGE 

!MESSAGE NMAKE /f "DrawStuff.mak".

!MESSAGE 

!MESSAGE You can specify a configuration when running NMAKE

!MESSAGE by defining the macro CFG on the command line. For example:

!MESSAGE 

!MESSAGE NMAKE /f "DrawStuff.mak" CFG="DrawStuff - Win32 Debug"

!MESSAGE 

!MESSAGE Possible choices for configuration are:

!MESSAGE 

!MESSAGE "DrawStuff - Win32 Release" (based on "Win32 (x86) Static Library")

!MESSAGE "DrawStuff - Win32 Debug" (based on "Win32 (x86) Static Library")

!MESSAGE 



# Begin Project

# PROP AllowPerConfigDependencies 0

# PROP Scc_ProjName ""

# PROP Scc_LocalPath ""

CPP=cl.exe

RSC=rc.exe



!IF  "$(CFG)" == "DrawStuff - Win32 Release"



# PROP BASE Use_MFC 0

# PROP BASE Use_Debug_Libraries 0

# PROP Use_MFC 0

# PROP Use_Debug_Libraries 0

# PROP Output_Dir "DrawStuff_Release"

# PROP Intermediate_Dir "DrawStuff_Release\Int"

# PROP Target_Dir ""

# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c

# ADD CPP /nologo /MD /W2 /GX /O2 /I "..\..\include" /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c

# ADD BASE RSC /l 0x409 /d "NDEBUG"

# ADD RSC /l 0x409 /d "NDEBUG"

BSC32=bscmake.exe

# ADD BASE BSC32 /nologo

# ADD BSC32 /nologo

LIB32=link.exe -lib

# ADD BASE LIB32 /nologo

# ADD LIB32 /nologo /out:"..\..\..\..\lib\DrawStuff.lib"



!ELSEIF  "$(CFG)" == "DrawStuff - Win32 Debug"



# PROP BASE Use_MFC 0

# PROP BASE Use_Debug_Libraries 1

# PROP Use_MFC 0

# PROP Use_Debug_Libraries 1

# PROP Output_Dir "DrawStuff_Debug"

# PROP Intermediate_Dir "DrawStuff_Debug\Int"

# PROP Target_Dir ""

# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c

# ADD CPP /nologo /MDd /W2 /Gm /GX /ZI /Od /I "..\..\include" /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /FR /YX /FD /GZ /c

# ADD BASE RSC /l 0x409 /d "_DEBUG"

# ADD RSC /l 0x409 /d "_DEBUG"

BSC32=bscmake.exe

# ADD BASE BSC32 /nologo

# ADD BSC32 /nologo

LIB32=link.exe -lib

# ADD BASE LIB32 /nologo

# ADD LIB32 /nologo /out:"..\..\..\..\lib\DrawStuffd.lib"



!ENDIF 



# Begin Target



# Name "DrawStuff - Win32 Release"

# Name "DrawStuff - Win32 Debug"

# Begin Group "Source Files"



# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"

# Begin Source File



SOURCE=..\..\drawstuff\src\drawstuff.cpp

# End Source File

# Begin Source File



SOURCE=..\..\drawstuff\src\resources.rc

# End Source File

# Begin Source File



SOURCE=..\..\drawstuff\src\windows.cpp

# End Source File

# End Group

# Begin Group "Header Files"



# PROP Default_Filter "h"

# Begin Source File



SOURCE=..\..\drawstuff\src\internal.h

# End Source File

# End Group

# End Target

# End Project

