# Microsoft Developer Studio Project File - Name="libbulletsoftbody" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=libbulletsoftbody - Win32 Release
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "libbulletsoftbody.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "libbulletsoftbody.mak" CFG="libbulletsoftbody - Win32 Release"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "libbulletsoftbody - Win32 DebugDoublePrecision" (based on "Win32 (x86) Static Library")
!MESSAGE "libbulletsoftbody - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "libbulletsoftbody - Win32 ReleaseDoublePrecision" (based on "Win32 (x86) Static Library")
!MESSAGE "libbulletsoftbody - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "libbulletsoftbody - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "release"
# PROP BASE Intermediate_Dir "release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\out\release6\build\libbulletsoftbody\"
# PROP Intermediate_Dir "..\..\out\release6\build\libbulletsoftbody\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /Gy /GF /MD /Ob2 /Og /Oi /Ot /Oy /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\libbulletsoftbody\libbulletsoftbody.res" /i "." /i "..\.." /i "..\..\src"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\release6\libs\libbulletsoftbody.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /OPT:NOREF /subsystem:windows  

!ELSEIF  "$(CFG)" == "libbulletsoftbody - Win32 ReleaseDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "release_dbl"
# PROP BASE Intermediate_Dir "release_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\release_dbl6\build\libbulletsoftbody\"
# PROP Intermediate_Dir "..\..\out\release_dbl6\build\libbulletsoftbody\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release_dbl6\build\libbulletsoftbody\libbulletsoftbody.res" /i "." /i "..\.." /i "..\..\src"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386  /subsystem:windows  

!ELSEIF  "$(CFG)" == "libbulletsoftbody - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "debug"
# PROP BASE Intermediate_Dir "debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\out\debug6\build\libbulletsoftbody\"
# PROP Intermediate_Dir "..\..\out\debug6\build\libbulletsoftbody\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /GR /MDd /ZI /Od /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\libbulletsoftbody\libbulletsoftbody.res" /i "." /i "..\.." /i "..\..\src"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\debug6\libs\libbulletsoftbody_d.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /subsystem:windows  

!ELSEIF  "$(CFG)" == "libbulletsoftbody - Win32 DebugDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "debug_dbl"
# PROP BASE Intermediate_Dir "debug_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\debug_dbl6\build\libbulletsoftbody\"
# PROP Intermediate_Dir "..\..\out\debug_dbl6\build\libbulletsoftbody\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug_dbl6\build\libbulletsoftbody\libbulletsoftbody.res" /i "." /i "..\.." /i "..\..\src"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386  /subsystem:windows  

!ENDIF 

# Begin Target

# Name "libbulletsoftbody - Win32 Release"
# Name "libbulletsoftbody - Win32 ReleaseDoublePrecision"
# Name "libbulletsoftbody - Win32 Debug"
# Name "libbulletsoftbody - Win32 DebugDoublePrecision"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftBody.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftBodyConcaveCollisionAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftBodyHelpers.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftBodyRigidBodyCollisionConfiguration.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftRigidCollisionAlgorithm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftRigidDynamicsWorld.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftSoftCollisionAlgorithm.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftBody.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftBodyConcaveCollisionAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftBodyHelpers.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftBodyInternals.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftBodyRigidBodyCollisionConfiguration.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftRigidCollisionAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftRigidDynamicsWorld.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSoftSoftCollisionAlgorithm.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletSoftBody\btSparseSDF.h
# End Source File
# End Group
# End Target
# End Project
