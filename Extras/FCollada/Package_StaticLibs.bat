rem Use this batch file to package up a new version of the FS import classes

rem Clean up the old version package files, if present
rm -f FCollada_StaticLibs.zip
rm -R -f Package_Temp

rem Copy all the interesting files: DLL, LIB, H, TXT over
rem to a temporary folder
mkdir Package_Temp
mkdir Package_Temp\FCDocument
mkdir Package_Temp\FMath
mkdir Package_Temp\FUtils
mkdir Package_Temp\libxml
mkdir Package_Temp\Documentation
copy Output\*.LIB Package_Temp
copy *.TXT Package_Temp
copy FCDocument\*.HPP Package_Temp\FCDocument
copy FCDocument\*.H Package_Temp\FCDocument
copy FMath\*.H Package_Temp\FMath
copy FMath\*.HPP Package_Temp\FMath
copy FUtils\*.H Package_Temp\FUtils
copy FUtils\*.HPP Package_Temp\FUtils
copy LibXML\include\libxml\*.H Package_Temp\libxml
copy FCollada.h Package_Temp
copy FColladaLib.vcproj Package_Temp
copy FColladaLib.cpp Package_Temp
copy Output\Doxygen\html\*.* Package_Temp\Documentation
copy Output\Doxygen\FCollada.html Package_Temp

rem Take out files that shouldn't be packaged
del Package_Temp\FCollada.LIB
del Package_Temp\FColladaD.LIB
del Package_Temp\FColladaU.LIB
del Package_Temp\FColladaUD.LIB
del Package_Temp\FMath\StdAfx.h
del Package_Temp\FUtils\StdAfx.h

rem Zip up the temporary directory
cd Package_Temp
zip -9 -r ..\FCollada_StaticLibs.zip .
cd ..

rem Clean up
rm -R -f Package_Temp
