# Microsoft Developer Studio Project File - Name="libglui" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=libglui - Win32 Release
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "libglui.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "libglui.mak" CFG="libglui - Win32 Release"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "libglui - Win32 DebugDoublePrecision" (based on "Win32 (x86) Static Library")
!MESSAGE "libglui - Win32 DebugDll" (based on "Win32 (x86) Static Library")
!MESSAGE "libglui - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "libglui - Win32 ReleaseDoublePrecision" (based on "Win32 (x86) Static Library")
!MESSAGE "libglui - Win32 ReleaseDll" (based on "Win32 (x86) Static Library")
!MESSAGE "libglui - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "libglui - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "release"
# PROP BASE Intermediate_Dir "release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\out\release6\build\libglui\"
# PROP Intermediate_Dir "..\..\out\release6\build\libglui\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /Gy /GF /MD /Ob2 /Og /Oi /Ot /Oy /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\libglui\libglui.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\release6\libs\libglui.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386 /OPT:NOREF /subsystem:windows /libpath:"..\..\Glut" 

!ELSEIF  "$(CFG)" == "libglui - Win32 ReleaseDll"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "release_dll"
# PROP BASE Intermediate_Dir "release_dll"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\release_dll6\build\libglui\"
# PROP Intermediate_Dir "..\..\out\release_dll6\build\libglui\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release_dll6\build\libglui\libglui.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386  /subsystem:windows /libpath:"..\..\Glut" 

!ELSEIF  "$(CFG)" == "libglui - Win32 ReleaseDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "release_dbl"
# PROP BASE Intermediate_Dir "release_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\release_dbl6\build\libglui\"
# PROP Intermediate_Dir "..\..\out\release_dbl6\build\libglui\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release_dbl6\build\libglui\libglui.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386  /subsystem:windows /libpath:"..\..\Glut" 

!ELSEIF  "$(CFG)" == "libglui - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "debug"
# PROP BASE Intermediate_Dir "debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\out\debug6\build\libglui\"
# PROP Intermediate_Dir "..\..\out\debug6\build\libglui\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /GR /MDd /ZI /Od /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\libglui\libglui.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\debug6\libs\libglui_d.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /subsystem:windows /libpath:"..\..\Glut" 

!ELSEIF  "$(CFG)" == "libglui - Win32 DebugDll"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "debug_dll"
# PROP BASE Intermediate_Dir "debug_dll"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\debug_dll6\build\libglui\"
# PROP Intermediate_Dir "..\..\out\debug_dll6\build\libglui\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug_dll6\build\libglui\libglui.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386  /subsystem:windows /libpath:"..\..\Glut" 

!ELSEIF  "$(CFG)" == "libglui - Win32 DebugDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "debug_dbl"
# PROP BASE Intermediate_Dir "debug_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\debug_dbl6\build\libglui\"
# PROP Intermediate_Dir "..\..\out\debug_dbl6\build\libglui\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug_dbl6\build\libglui\libglui.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386  /subsystem:windows /libpath:"..\..\Glut" 

!ENDIF 

# Begin Target

# Name "libglui - Win32 Release"
# Name "libglui - Win32 ReleaseDll"
# Name "libglui - Win32 ReleaseDoublePrecision"
# Name "libglui - Win32 Debug"
# Name "libglui - Win32 DebugDll"
# Name "libglui - Win32 DebugDoublePrecision"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Extras\glui\algebra3.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\arcball.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_add_controls.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_bitmap_img_data.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_bitmaps.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_button.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_checkbox.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_column.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_commandline.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_control.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_edittext.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_filebrowser.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_list.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_listbox.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_mouse_iaction.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_node.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_panel.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_radio.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_rollout.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_rotation.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_scrollbar.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_separator.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_spinner.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_statictext.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_string.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_textbox.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_translation.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_tree.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_treepanel.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_window.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\quaternion.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Extras\glui\algebra3.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\arcball.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_internal.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\glui_internal_control.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\glui\quaternion.h
# End Source File
# End Group
# End Target
# End Project
