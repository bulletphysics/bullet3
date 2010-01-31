; example1.nsi
;
; This script is perhaps one of the simplest NSIs you can make. All of the
; optional settings are left to their default settings. The installer simply
; prompts the user asking them where to install, and drops a copy of "MyProg.exe"
; there.

;--------------------------------

; The name of the installer
Name "Dynamica Bullet 2.76 physics plugin for Maya 2009"

; The file to write
OutFile "DynamicaForMaya2009_64bit.exe"


; The default installation directory
InstallDir $PROGRAMFILES\DynamicaBullet2.76\

UninstPage uninstConfirm
UninstPage instfiles

; The text to prompt the user to enter a directory
DirText "This will install Dynamica Bullet For Maya. Choose destination directory"

;--------------------------------

; The stuff to install
Section "" ;No components page, name is not important
;Create Dynamica directories
;CreateDirectory "$INSTDIR\dll"
CreateDirectory "$INSTDIR\doc"	
CreateDirectory "$INSTDIR\scenes\"
CreateDirectory "$INSTDIR\icons"
CreateDirectory "$INSTDIR\plug-ins"	
CreateDirectory "$INSTDIR\scripts"	

;SetOutPath "$INSTDIR\dll"
;File "dll\*.dll"
SetOutPath "$INSTDIR\doc"
File "doc\*.*"
SetOutPath "$INSTDIR\scenes"
File "scenes\*.*"
SetOutPath "$INSTDIR\icons"
File "icons\*.*"
SetOutPath "$INSTDIR\plug-ins"
File "BulletMayaPlugin.mll"
;File "C:\Program Files\Microsoft Visual Studio 8\VC\redist\x86\Microsoft.VC80.CRT\msvcp80.dll"
;File "C:\Program Files\Microsoft Visual Studio 8\VC\redist\x86\Microsoft.VC80.CRT\msvcr80.dll"

File "C:\Program Files (x86)\Microsoft Visual Studio 9.0\VC\redist\x86\Microsoft.VC90.CRT\msvcp90.dll"
File "C:\Program Files (x86)\Microsoft Visual Studio 9.0\VC\redist\x86\Microsoft.VC90.CRT\msvcr90.dll"

SetOutPath "$INSTDIR\scripts"
File "scripts\*.*"
SetOutPath	"$DOCUMENTS\maya\modules\"
File "BulletDynamica.6_module"
	
FileOpen $0 $DOCUMENTS\maya\modules\BulletDynamica.6_module a
FileSeek $0 0 END
FileWrite $0 "$INSTDIR$\n"
FileClose $0

CreateDirectory "$SMPROGRAMS\Dynamica Bullet"
CreateShortCut "$SMPROGRAMS\Dynamica Bullet\Documentation.lnk" "$INSTDIR\doc\index.html"
CreateShortCut "$SMPROGRAMS\Dynamica Bullet\Examples.lnk" "$INSTDIR\scenes\"
CreateShortCut "$SMPROGRAMS\Dynamica Bullet\Uninstall.lnk" "$INSTDIR\Uninstall.exe"
ExecShell "open" "$INSTDIR\doc\index.html"

WriteUninstaller $INSTDIR\Uninstall.exe
SectionEnd ; end the section

Section "Uninstall"
	ClearErrors
	MessageBox MB_YESNO "Uninstall Bullet for MAYA?" IDNO end
	
	Delete "$DOCUMENTS\maya\modules\DynamicaBullet.6_module"
	RMDir /r "$SMPROGRAMS\Dynamica Bullet\"
	RMDir /r "$INSTDIR"
	end:
SectionEnd