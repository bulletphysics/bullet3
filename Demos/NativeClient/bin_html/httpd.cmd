@echo off
setlocal

REM Relative path of CygWin
set CYGWIN=%~dp0%..\third_party\cygwin\bin

PATH=%CYGWIN%;%PATH%

python httpd.py
