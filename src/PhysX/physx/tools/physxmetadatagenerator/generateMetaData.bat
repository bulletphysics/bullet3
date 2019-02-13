::
:: Redistribution and use in source and binary forms, with or without
:: modification, are permitted provided that the following conditions
:: are met:
::  * Redistributions of source code must retain the above copyright
::    notice, this list of conditions and the following disclaimer.
::  * Redistributions in binary form must reproduce the above copyright
::    notice, this list of conditions and the following disclaimer in the
::    documentation and/or other materials provided with the distribution.
::  * Neither the name of NVIDIA CORPORATION nor the names of its
::    contributors may be used to endorse or promote products derived
::    from this software without specific prior written permission.
::
:: THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
:: EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
:: IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
:: PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
:: CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
:: EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
:: PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
:: PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
:: OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
:: (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
:: OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
::
:: Copyright (c) 2016-2018 NVIDIA Corporation. All rights reserved.

@echo off
setlocal EnableDelayedExpansion 

:: batch script that runs generateMetaData.py with either
::
:: -environment variable PYTHON
:: -p4sw location of python
:: -python.exe in PATH 
::
:: see readme.txt

cd %~dp0

:: Get PxShared path from packman, if packman available:
set PACKMAN_CMD=..\..\buildtools\packman\packman
if not exist !PACKMAN_CMD! goto no_packman

call "..\..\buildtools\update_packman.cmd"
@if errorlevel 1 @exit /b %errorlevel%

call !PACKMAN_CMD! pull "%~dp0..\..\dependencies.xml" --include-tag=RequiredForMetaGen
@if errorlevel 1 @exit /b %errorlevel%
:no_packman

:: look for python in p4 location unless PYTHON is set
if not defined PYTHON (
	call :find_root_path %~p0 "tools\python\3.3" _RESULT
	if not !_RESULT!==0 (
		set PYTHON=!_RESULT!\tools\python\3.3\python.exe
	)
)

:: look for python in PATH unless PYTHON is set
if not defined PYTHON (
	where python.exe > nul 2>&1
	IF !ERRORLEVEL! == 0 (
		set PYTHON=python.exe
	)
)

if not defined PYTHON (
	goto no_python
)

echo using: %PYTHON%

:: visual studio 2015 is required for the meta data generation
:: run vcvarsall.bat to get visual studio developer console environment variables
if not defined VS140COMNTOOLS (
	goto no_vs140
)
		
echo calling: %VS140COMNTOOLS%..\..\VC\vcvarsall.bat
call "%VS140COMNTOOLS%..\..\VC\vcvarsall.bat"
%PYTHON% generateMetaData.py %1	

endlocal
exit /b %ERRORLEVEL%

:no_python
echo no python found, please set PYTHON environment variable to python.exe path, or make sure python.exe is in PATH.
endlocal
exit /b 1

:no_vs140
echo echo make sure vs 2015 is installed.
endlocal
exit /b 1


:: **************************************************************************
:: functions
:: **************************************************************************

:: find a root directory containing a known directory (as a hint)
:find_root_path
	setlocal
	set START_DIR=%~1
	set CONTAINED_DIR=%~2
	
	:: search directory tree
	set TMP_DIR=!START_DIR!
	set OUT_DIR=0
	:find_root_path_loop
	if exist !TMP_DIR!\!CONTAINED_DIR! goto :find_root_path_loop_end
	set TMP_DIR=!TMP_DIR!..\
	
	:: normalize path
	pushd !TMP_DIR!
	set OUT_DIR=!CD!
	popd
	
	:: break if we reach the root, by checking the last two charactors
	if "!OUT_DIR:~-2!"==":\" (
		set OUT_DIR=0
		goto :find_root_path_loop_end
	)
	
	goto :find_root_path_loop
	:find_root_path_loop_end
		
	::echo no idea why we need to use % here
	endlocal & set _RESULT=%OUT_DIR%
	exit /b 0

