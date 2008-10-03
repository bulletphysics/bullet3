#----------------------------------------------------------------------------
# Autoconf input script. Invoke the ./autogen.sh script to generate a
# configure script from this file.
#----------------------------------------------------------------------------
AC_PREREQ([2.54])

#----------------------------------------------------------------------------
# Initialize Autoconf.
#----------------------------------------------------------------------------
AC_INIT(
    [bullet],
    [2.72],
    [bullet@erwincoumans.com])
AC_CANONICAL_HOST
CS_PACKAGEINFO(
    [Bullet Continuous Collision Detection and Physics Library],
    [Copyright (c) 2005-2008  Erwin Coumans],
    [http://www.bulletphysics.com])
AC_CONFIG_SRCDIR([configure.ac])
AM_INIT_AUTOMAKE
AC_PROG_CC
AC_PROG_CXX
AC_PROG_LIBTOOL

case "$host" in
        *-*-mingw*|*-*-cygwin*)
                AC_DEFINE(PLATFORM_WIN32, 1, [Platform is Win32])
                opengl_LIBS="-lunsupported_platform"
                PLATFORM_STRING="Win32"
                ;;
        *-*-linux*)
                AC_DEFINE(PLATFORM_LINUX, 1, [Platform is Linux])
                opengl_LIBS="-lGL -lGLU -lglut"
                PLATFORM_STRING="Linux"
                ;;
        *-*-darwin*)
                AC_DEFINE(PLATFORM_APPLE, 1, [Platform is Apple])
                opengl_LIBS="-framework AGL -framework OpenGL -framework GLUT"
                PLATFORM_STRING="Apple"
                ;;
        *)
                AC_MSG_WARN([*** Please add $host to configure.ac checks!])
                ;;
esac
AC_SUBST(opengl_LIBS)

case "$host" in
        i?86-* | k?-* | athlon-* | pentium*-)
                AC_DEFINE(ARCH_X86, 1, [Architecture is x86])
                ARCH_SPECIFIC_CFLAGS="-mmmx -msse"
                ARCH_STRING="X86"
                ;;
        x86_64-*)
                AC_DEFINE(ARCH_X86_64, 1, [Architecture is x86-64])
                ARCH_SPECIFIC_CFLAGS="-mmmx -msse"
                ARCH_STRING="X86-64"
                ;;
        ppc-* | powerpc-*)
                AC_DEFINE(ARCH_PPC, 1, [Architecture is PowerPC])
                ARCH_SPECIFIC_CFLAGS="-maltivec"
                ARCH_STRING="PowerPC"
                ;;
        *)
                AC_MSG_ERROR([Unknown Architecture])
                ;;
esac
AC_C_BIGENDIAN


#----------------------------------------------------------------------------
# Setup for the configuration header.
#----------------------------------------------------------------------------
AC_CONFIG_HEADERS([config.h])

#----------------------------------------------------------------------------
# Check for tools.
#----------------------------------------------------------------------------
CS_PROG_CC
AS_IF([test -z "$CC"],
    [AC_MSG_ERROR([Could not find a usable C compiler.])])
CS_PROG_CXX
AS_IF([test -z "$CXX"],
    [AC_MSG_ERROR([Could not find a usable C++ compiler.])])
CS_PROG_LINK

CS_CHECK_COMMON_TOOLS_LINK
CS_CHECK_COMMON_TOOLS_BASIC
CS_CHECK_COMMON_TOOLS_DOC_DOXYGEN

CS_CHECK_PROGS([PERL], [perl5 perl])
CS_EMIT_BUILD_PROPERTY([PERL], [$PERL])

CS_CHECK_TEMPLATE_TOOLKIT2([emit])

#----------------------------------------------------------------------------
# Check if C++ exceptions can be disabled.
#----------------------------------------------------------------------------
CS_EMIT_BUILD_FLAGS([how to disable C++ exceptions],
    [cs_cv_prog_cxx_disable_exceptions], [CS_CREATE_TUPLE([-fno-exceptions])],
    [C++], [COMPILER.C++FLAGS.EXCEPTIONS.DISABLE], [],
    [CS_EMIT_BUILD_PROPERTY([COMPILER.C++FLAGS],
	[$cs_cv_prog_cxx_disable_exceptions], [+])])


#----------------------------------------------------------------------------
# Determine system type
#----------------------------------------------------------------------------
CS_CHECK_HOST

#----------------------------------------------------------------------------
# Check for syntax problems / header files
#----------------------------------------------------------------------------
# Nothing yet.

#----------------------------------------------------------------------------
# Check for GLUT.
#----------------------------------------------------------------------------
AS_IF([test $cs_host_family = windows],
    [# Tack the GLUT that comes with bullet onto compiler & linker flags.
    _AC_SRCDIRS(["."])
    glut_cflags="-I$ac_top_srcdir/Glut"
    glut_lflags="-L$ac_top_srcdir/Glut"
    CFLAGS="$CFLAGS $glut_cflags"
    LDFLAGS="$LDFLAGS $glut_lflags"
    CS_EMIT_BUILD_PROPERTY([COMPILER.CFLAGS], [$glut_cflags], [+])
    CS_EMIT_BUILD_PROPERTY([COMPILER.LFLAGS], [$glut_lflags], [+])
    ])
CS_CHECK_GLUT


#----------------------------------------------------------------------------
# Package configuration switches.
#----------------------------------------------------------------------------
AC_ARG_ENABLE([debug],
    [AC_HELP_STRING([--enable-debug],
	[build with debugging information (default NO)])],
    [], [enable_debug=no])

AC_MSG_CHECKING([build mode])
AS_IF([test $enable_debug = yes], [build_mode=debug], [build_mode=optimize])
AC_MSG_RESULT([$build_mode])

CS_EMIT_BUILD_PROPERTY([MODE], [$build_mode])

#-----------------------------------------------------------------------------
# Emit install paths and package information.
#-----------------------------------------------------------------------------
CS_OUTPUT_INSTALLDIRS
CS_EMIT_PACKAGEINFO


CXXFLAGS="$CXXFLAGS $CFLAGS"
#----------------------------------------------------------------------------
# Emit generated files.
#----------------------------------------------------------------------------
CS_JAMCONFIG_OUTPUT([Jamconfig])
AC_CONFIG_FILES([bullet.pc Jamfile Makefile Demos/MultiThreadedDemo/Makefile Demos/ColladaDemo/Makefile Demos/OpenGL/Makefile Demos/BasicDemo/Makefile Demos/CcdPhysicsDemo/Makefile Demos/VehicleDemo/Makefile src/Makefile Extras/Makefile])
AC_OUTPUT

AC_MSG_NOTICE([
You can type 'make' or 'jam' to build Bullet.
Alternatively, you can use cmake or use the wksbullet.sln visual studio x solutions in the msvc/x folder.

CMake home:http://cmake.org
Jam home: http://www.perforce.com/jam/jam.html
Jam source: ftp://ftp.perforce.com/jam/

Please type 'make' to build Bullet
])
