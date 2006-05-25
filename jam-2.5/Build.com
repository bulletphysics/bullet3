! Bootstrap build script for Jam
$ cxx /define=VMS builtins.c
$ cxx /define=VMS command.c
$ cxx /define=VMS compile.c
$ cxx /define=VMS expand.c
$ cxx /define=VMS execvms.c
$ cxx /define=VMS filevms.c
$ cxx /define=VMS glob.c
$ cxx /define=VMS hash.c
$ cxx /define=VMS headers.c
$ cxx /define=VMS jambase.c
$ cxx /define=VMS lists.c
$ cxx /define=VMS make.c
$ cxx /define=VMS make1.c
$ cxx /define=VMS newstr.c
$ cxx /define=VMS option.c
$ cxx /define=VMS parse.c
$ cxx /define=VMS pathvms.c
$ cxx /define=VMS regexp.c
$ cxx /define=VMS rules.c
$ cxx /define=VMS scan.c
$ cxx /define=VMS search.c
$ cxx /define=VMS timestamp.c
$ cxx /define=VMS variable.c
$ cxx /define=VMS jam.c
$ cxx /define=VMS jamgram.c
$ cxxlink/exe=jam.exe command.obj, compile.obj, execvms.obj, expand.obj, -
    filevms.obj, glob.obj, hash.obj, headers.obj, lists.obj, make.obj, -
    make1.obj, newstr.obj, option.obj, parse.obj, pathvms.obj, regexp.obj, -
    rules.obj, scan.obj, search.obj, timestamp.obj, variable.obj, jam.obj, -
    jamgram.obj, jambase.obj, builtins.obj
