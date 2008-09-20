# Microsoft Developer Studio Project File - Name="liblibxml" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=liblibxml - Win32 Release
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "liblibxml.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "liblibxml.mak" CFG="liblibxml - Win32 Release"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "liblibxml - Win32 DebugDoublePrecision" (based on "Win32 (x86) Static Library")
!MESSAGE "liblibxml - Win32 DebugDll" (based on "Win32 (x86) Static Library")
!MESSAGE "liblibxml - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "liblibxml - Win32 ReleaseDoublePrecision" (based on "Win32 (x86) Static Library")
!MESSAGE "liblibxml - Win32 ReleaseDll" (based on "Win32 (x86) Static Library")
!MESSAGE "liblibxml - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "liblibxml - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "release"
# PROP BASE Intermediate_Dir "release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\out\release6\build\liblibxml\"
# PROP Intermediate_Dir "..\..\out\release6\build\liblibxml\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /Gy /GF /MD /Ob2 /Og /Oi /Ot /Oy /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Extras\LibXML" /I "..\..\Extras\LibXML\include"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\liblibxml\liblibxml.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Extras\LibXML" /i "..\..\Extras\LibXML\include"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\release6\libs\liblibxml.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /OPT:NOREF /subsystem:windows  

!ELSEIF  "$(CFG)" == "liblibxml - Win32 ReleaseDll"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "release_dll"
# PROP BASE Intermediate_Dir "release_dll"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\release_dll6\build\liblibxml\"
# PROP Intermediate_Dir "..\..\out\release_dll6\build\liblibxml\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Extras\LibXML" /I "..\..\Extras\LibXML\include"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release_dll6\build\liblibxml\liblibxml.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Extras\LibXML" /i "..\..\Extras\LibXML\include"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386  /subsystem:windows  

!ELSEIF  "$(CFG)" == "liblibxml - Win32 ReleaseDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "release_dbl"
# PROP BASE Intermediate_Dir "release_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\release_dbl6\build\liblibxml\"
# PROP Intermediate_Dir "..\..\out\release_dbl6\build\liblibxml\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Extras\LibXML" /I "..\..\Extras\LibXML\include"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release_dbl6\build\liblibxml\liblibxml.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Extras\LibXML" /i "..\..\Extras\LibXML\include"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386  /subsystem:windows  

!ELSEIF  "$(CFG)" == "liblibxml - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "debug"
# PROP BASE Intermediate_Dir "debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\out\debug6\build\liblibxml\"
# PROP Intermediate_Dir "..\..\out\debug6\build\liblibxml\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /GR /MDd /ZI /Od /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Extras\LibXML" /I "..\..\Extras\LibXML\include"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\liblibxml\liblibxml.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Extras\LibXML" /i "..\..\Extras\LibXML\include"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\debug6\libs\liblibxml_d.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /subsystem:windows  

!ELSEIF  "$(CFG)" == "liblibxml - Win32 DebugDll"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "debug_dll"
# PROP BASE Intermediate_Dir "debug_dll"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\debug_dll6\build\liblibxml\"
# PROP Intermediate_Dir "..\..\out\debug_dll6\build\liblibxml\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Extras\LibXML" /I "..\..\Extras\LibXML\include"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug_dll6\build\liblibxml\liblibxml.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Extras\LibXML" /i "..\..\Extras\LibXML\include"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386  /subsystem:windows  

!ELSEIF  "$(CFG)" == "liblibxml - Win32 DebugDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "debug_dbl"
# PROP BASE Intermediate_Dir "debug_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\debug_dbl6\build\liblibxml\"
# PROP Intermediate_Dir "..\..\out\debug_dbl6\build\liblibxml\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Extras\LibXML" /I "..\..\Extras\LibXML\include"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug_dbl6\build\liblibxml\liblibxml.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Extras\LibXML" /i "..\..\Extras\LibXML\include"
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

# Name "liblibxml - Win32 Release"
# Name "liblibxml - Win32 ReleaseDll"
# Name "liblibxml - Win32 ReleaseDoublePrecision"
# Name "liblibxml - Win32 Debug"
# Name "liblibxml - Win32 DebugDll"
# Name "liblibxml - Win32 DebugDoublePrecision"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Extras\LibXML\c14n.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\catalog.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\chvalid.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\debugXML.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\dict.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\DOCBparser.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\encoding.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\entities.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\error.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\globals.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\hash.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\HTMLparser.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\HTMLtree.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\legacy.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\list.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\nanoftp.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\nanohttp.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\parser.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\parserInternals.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\pattern.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\relaxng.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\SAX.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\SAX2.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\schematron.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\threads.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\tree.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\trio.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\trionan.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\triostr.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\uri.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\valid.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xinclude.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xlink.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xmlIO.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xmlmemory.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xmlmodule.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xmlreader.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xmlregexp.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xmlsave.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xmlschemas.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xmlschemastypes.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xmlstring.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xmlunicode.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xmlwriter.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xpath.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xpointer.c
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Extras\LibXML\acconfig.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\config-ps3.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\config-win32.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\config.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\config2.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\elfgcchack.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\libxml.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\trio.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\triodef.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\trionan.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\triop.h
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\triostr.h
# End Source File
# End Group
# End Target
# End Project
