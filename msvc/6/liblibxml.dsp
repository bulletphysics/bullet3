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
!MESSAGE "liblibxml - Win32 Debug" (based on "Win32 (x86) Static Library")
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
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /Gy /GF /MD /Ob2 /Og /Oi /Ot /Oy /D "_LIB" /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\Bullet" /I "..\..\BulletDynamics" /I "..\..\LinearMath" /I "..\..\Extras\PhysicsInterface\Common" /I "..\..\Extras\ConvexDecomposition" /I "..\..\Extras\COLLADA_DOM\include" /I "..\..\Extras\COLLADA_DOM\include\1.4" /I "..\..\Extras\LibXML\include"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\liblibxml\liblibxml.res" /i "." /i "..\.." /i "..\..\Bullet" /i "..\..\BulletDynamics" /i "..\..\LinearMath" /i "..\..\Extras\PhysicsInterface\Common" /i "..\..\Extras\ConvexDecomposition" /i "..\..\Extras\COLLADA_DOM\include" /i "..\..\Extras\COLLADA_DOM\include\1.4" /i "..\..\Extras\LibXML\include"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\release6\libs\liblibxml.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /OPT:NOREF /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

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
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /GR /MDd /ZI /Od /D "_LIB" /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_LIB" /D "_WINDOWS" /D "WIN32"  /I "." /I "..\.." /I "..\..\Bullet" /I "..\..\BulletDynamics" /I "..\..\LinearMath" /I "..\..\Extras\PhysicsInterface\Common" /I "..\..\Extras\ConvexDecomposition" /I "..\..\Extras\COLLADA_DOM\include" /I "..\..\Extras\COLLADA_DOM\include\1.4" /I "..\..\Extras\LibXML\include"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_LIB" /D "_WINDOWS"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\liblibxml\liblibxml.res" /i "." /i "..\.." /i "..\..\Bullet" /i "..\..\BulletDynamics" /i "..\..\LinearMath" /i "..\..\Extras\PhysicsInterface\Common" /i "..\..\Extras\ConvexDecomposition" /i "..\..\Extras\COLLADA_DOM\include" /i "..\..\Extras\COLLADA_DOM\include\1.4" /i "..\..\Extras\LibXML\include"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\out\debug6\libs\liblibxml_d.lib"
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib   /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /subsystem:windows  
# Begin Special Build Tool
SOURCE="$(InputPath)"
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "liblibxml - Win32 Release"
# Name "liblibxml - Win32 Debug"
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

SOURCE=..\..\Extras\LibXML\runsuite.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\runtest.c
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

SOURCE=..\..\Extras\LibXML\xmlcatalog.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xmlIO.c
# End Source File
# Begin Source File

SOURCE=..\..\Extras\LibXML\xmllint.c
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
