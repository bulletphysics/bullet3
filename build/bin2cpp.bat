
rem @echo off


premake4 --file=bin2cpp.lua --binfile="../btgui/FontFiles/OpenSans.ttf" --cppfile="../btgui/FontFiles/OpenSans.cpp" --stringname="OpenSansData" bin2cpp

pause