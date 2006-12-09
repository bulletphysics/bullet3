# Microsoft Developer Studio Project File - Name="libconvexdecomposition" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=libconvexdecomposition - Win32 Release
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "libconvexdecomposition.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "libconvexdecomposition.mak" CFG="libconvexdecomposition - Win32 Release"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "libconvexdecomposition - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "libconvexdecomposition - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "libconvexdecomposition - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "release"
# PROP BASE Intermediate_Dir "release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\out\release6\build\libconvexdecomposition\"
# PROP Intermediate_Dir "..\..\out\release6\build\libconvexdecomposition\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /Gy /GF /MD /Ob2 /Og /Oi /Ot /Oy /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\libconvexdecomposition\libconvexdecomposition.res" /i "." /i "..\.." /i "..\..\src"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\release6\libs\libconvexdecomposition.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /OPT:NOREF /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ELSEIF  "$(CFG)" == "libconvexdecomposition - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "debug"
# PROP BASE Intermediate_Dir "debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\out\debug6\build\libconvexdecomposition\"
# PROP Intermediate_Dir "..\..\out\debug6\build\libconvexdecomposition\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /GR /MDd /ZI /Od /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\libconvexdecomposition\libconvexdecomposition.res" /i "." /i "..\.." /i "..\..\src"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\debug6\libs\libconvexdecomposition_d.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "libconvexdecomposition - Win32 Release"
# Name "libconvexdecomposition - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\bestfit.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\bestfitobb.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\cd_hull.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\cd_wavefront.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\concavity.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\ConvexBuilder.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\ConvexDecomposition.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\fitsphere.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\float_math.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\meshvolume.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\planetri.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\raytri.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\splitplane.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\vlookup.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\bestfit.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\bestfitobb.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\cd_hull.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\cd_vector.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\cd_wavefront.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\concavity.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\ConvexBuilder.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\ConvexDecomposition.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\fitsphere.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\float_math.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\meshvolume.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\planetri.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\raytri.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\splitplane.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\ConvexDecomposition\vlookup.h
# End Source File
# End Group
# End Target
# End Project
