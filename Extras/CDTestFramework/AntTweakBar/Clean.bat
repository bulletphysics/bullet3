RMDIR /S /Q src\debug
RMDIR /S /Q src\release
CD src
DEL *.ncb *.aps *.o *.bak
DEL /A:h *.suo
CD ..
RMDIR /S /Q lib\debug
RMDIR /S /Q examples\debug
CD examples
DEL *.ncb *.aps *.o *.bak
DEL /A:h *.suo
DEL /S BuildLog.htm
DEL bin\*.obj
DEL bin\*.idb
CD ..

PAUSE


