# Microsoft Developer Studio Project File - Name="libbulletmultithreaded" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=libbulletmultithreaded - Win32 Release
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "libbulletmultithreaded.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "libbulletmultithreaded.mak" CFG="libbulletmultithreaded - Win32 Release"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "libbulletmultithreaded - Win32 DebugDoublePrecision" (based on "Win32 (x86) Static Library")
!MESSAGE "libbulletmultithreaded - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "libbulletmultithreaded - Win32 ReleaseDoublePrecision" (based on "Win32 (x86) Static Library")
!MESSAGE "libbulletmultithreaded - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "libbulletmultithreaded - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "release"
# PROP BASE Intermediate_Dir "release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\out\release6\build\libbulletmultithreaded\"
# PROP Intermediate_Dir "..\..\out\release6\build\libbulletmultithreaded\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /Gy /GF /MD /Ob2 /Og /Oi /Ot /Oy /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Extras\BulletMultiThreaded"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\libbulletmultithreaded\libbulletmultithreaded.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Extras\BulletMultiThreaded"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\release6\libs\libbulletmultithreaded.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /OPT:NOREF /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ELSEIF  "$(CFG)" == "libbulletmultithreaded - Win32 ReleaseDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "release_dbl"
# PROP BASE Intermediate_Dir "release_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\release_dbl6\build\libbulletmultithreaded\"
# PROP Intermediate_Dir "..\..\out\release_dbl6\build\libbulletmultithreaded\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Extras\BulletMultiThreaded"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release_dbl6\build\libbulletmultithreaded\libbulletmultithreaded.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Extras\BulletMultiThreaded"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386  /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ELSEIF  "$(CFG)" == "libbulletmultithreaded - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "debug"
# PROP BASE Intermediate_Dir "debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\out\debug6\build\libbulletmultithreaded\"
# PROP Intermediate_Dir "..\..\out\debug6\build\libbulletmultithreaded\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /GR /MDd /ZI /Od /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Extras\BulletMultiThreaded"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\libbulletmultithreaded\libbulletmultithreaded.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Extras\BulletMultiThreaded"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\debug6\libs\libbulletmultithreaded_d.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ELSEIF  "$(CFG)" == "libbulletmultithreaded - Win32 DebugDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "debug_dbl"
# PROP BASE Intermediate_Dir "debug_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\debug_dbl6\build\libbulletmultithreaded\"
# PROP Intermediate_Dir "..\..\out\debug_dbl6\build\libbulletmultithreaded\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Extras\BulletMultiThreaded"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug_dbl6\build\libbulletmultithreaded\libbulletmultithreaded.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Extras\BulletMultiThreaded"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386  /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "libbulletmultithreaded - Win32 Release"
# Name "libbulletmultithreaded - Win32 ReleaseDoublePrecision"
# Name "libbulletmultithreaded - Win32 Debug"
# Name "libbulletmultithreaded - Win32 DebugDoublePrecision"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\btThreadSupportInterface.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuCollisionTaskProcess.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuContactManifoldCollisionAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuFakeDma.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuGatheringCollisionDispatcher.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuLibspe2Support.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuNarrowPhaseCollisionTask\SpuContactResult.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuNarrowPhaseCollisionTask\SpuGatheringCollisionTask.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuNarrowPhaseCollisionTask\SpuGjkPairDetector.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuNarrowPhaseCollisionTask\SpuMinkowskiPenetrationDepthSolver.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuNarrowPhaseCollisionTask\SpuVoronoiSimplexSolver.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuSampleTaskProcess.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\Win32ThreadSupport.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuNarrowPhaseCollisionTask\SpuContactResult.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuNarrowPhaseCollisionTask\SpuConvexPenetrationDepthSolver.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuNarrowPhaseCollisionTask\SpuGatheringCollisionTask.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuNarrowPhaseCollisionTask\SpuGjkPairDetector.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuNarrowPhaseCollisionTask\SpuLocalSupport.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuNarrowPhaseCollisionTask\SpuMinkowskiPenetrationDepthSolver.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\BulletMultiThreaded\SpuNarrowPhaseCollisionTask\SpuVoronoiSimplexSolver.h
# End Source File
# End Group
# End Target
# End Project
