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
!MESSAGE "libbulletdynamics - Win32 DebugDoublePrecision" (based on "Win32 (x86) Static Library")
!MESSAGE "libbulletdynamics - Win32 DebugDll" (based on "Win32 (x86) Static Library")
!MESSAGE "libbulletdynamics - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "libbulletdynamics - Win32 ReleaseDoublePrecision" (based on "Win32 (x86) Static Library")
!MESSAGE "libbulletdynamics - Win32 ReleaseDll" (based on "Win32 (x86) Static Library")
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
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /Gy /GF /MD /Ob2 /Zm1000 /Og /Oi /Ot /Oy /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\libbulletdynamics\libbulletdynamics.res" /i "." /i "..\.." /i "..\..\src"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\release6\libs\libbulletdynamics.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /OPT:NOREF /subsystem:windows  

!ELSEIF  "$(CFG)" == "libbulletdynamics - Win32 ReleaseDll"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "release_dll"
# PROP BASE Intermediate_Dir "release_dll"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\release_dll6\build\libbulletdynamics\"
# PROP Intermediate_Dir "..\..\out\release_dll6\build\libbulletdynamics\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release_dll6\build\libbulletdynamics\libbulletdynamics.res" /i "." /i "..\.." /i "..\..\src"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386  /subsystem:windows  

!ELSEIF  "$(CFG)" == "libbulletdynamics - Win32 ReleaseDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "release_dbl"
# PROP BASE Intermediate_Dir "release_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\release_dbl6\build\libbulletdynamics\"
# PROP Intermediate_Dir "..\..\out\release_dbl6\build\libbulletdynamics\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release_dbl6\build\libbulletdynamics\libbulletdynamics.res" /i "." /i "..\.." /i "..\..\src"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386  /subsystem:windows  

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
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /GR /MDd /Zm1000 /ZI /Od /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\libbulletdynamics\libbulletdynamics.res" /i "." /i "..\.." /i "..\..\src"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\debug6\libs\libbulletdynamics_d.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /subsystem:windows  

!ELSEIF  "$(CFG)" == "libbulletdynamics - Win32 DebugDll"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "debug_dll"
# PROP BASE Intermediate_Dir "debug_dll"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\debug_dll6\build\libbulletdynamics\"
# PROP Intermediate_Dir "..\..\out\debug_dll6\build\libbulletdynamics\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug_dll6\build\libbulletdynamics\libbulletdynamics.res" /i "." /i "..\.." /i "..\..\src"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386  /subsystem:windows  

!ELSEIF  "$(CFG)" == "libbulletdynamics - Win32 DebugDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "debug_dbl"
# PROP BASE Intermediate_Dir "debug_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\debug_dbl6\build\libbulletdynamics\"
# PROP Intermediate_Dir "..\..\out\debug_dbl6\build\libbulletdynamics\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug_dbl6\build\libbulletdynamics\libbulletdynamics.res" /i "." /i "..\.." /i "..\..\src"
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

# Name "libbulletdynamics - Win32 Release"
# Name "libbulletdynamics - Win32 ReleaseDll"
# Name "libbulletdynamics - Win32 ReleaseDoublePrecision"
# Name "libbulletdynamics - Win32 Debug"
# Name "libbulletdynamics - Win32 DebugDll"
# Name "libbulletdynamics - Win32 DebugDoublePrecision"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Character\btKinematicCharacterController.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btConeTwistConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btContactConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btGeneric6DofConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btGeneric6DofSpringConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btHinge2Constraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btHingeConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btPoint2PointConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btSequentialImpulseConstraintSolver.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btSliderConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btSolve2LinearConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btTypedConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btUniversalConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Dynamics\btContinuousDynamicsWorld.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Dynamics\btDiscreteDynamicsWorld.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Dynamics\btRigidBody.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Dynamics\btSimpleDynamicsWorld.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Dynamics\Bullet-C-API.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Vehicle\btRaycastVehicle.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Vehicle\btWheelInfo.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Character\btCharacterControllerInterface.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Character\btKinematicCharacterController.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btConeTwistConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btConstraintSolver.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btContactConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btContactSolverInfo.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btGeneric6DofConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btGeneric6DofSpringConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btHinge2Constraint.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btHingeConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btJacobianEntry.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btPoint2PointConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btSequentialImpulseConstraintSolver.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btSliderConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btSolve2LinearConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btSolverBody.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btSolverConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btTypedConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\ConstraintSolver\btUniversalConstraint.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Dynamics\btActionInterface.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Dynamics\btContinuousDynamicsWorld.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Dynamics\btDiscreteDynamicsWorld.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Dynamics\btDynamicsWorld.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Dynamics\btRigidBody.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Dynamics\btSimpleDynamicsWorld.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Vehicle\btRaycastVehicle.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Vehicle\btVehicleRaycaster.h
# End Source File
# Begin Source File

SOURCE=..\..\src\BulletDynamics\Vehicle\btWheelInfo.h
# End Source File
# End Group
# End Target
# End Project
