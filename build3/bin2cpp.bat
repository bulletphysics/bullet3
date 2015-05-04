
rem @echo off


premake4 --file=bin2cpp.lua --binfile="../btgui/OpenGLWindow/OpenSans.ttf" --cppfile="../btgui/OpenGLWindow/OpenSans.cpp" --stringname="OpenSansData" bin2cpp

pause
