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
    [2.77],
    [bullet@erwincoumans.com])
AC_CANONICAL_HOST
AC_CONFIG_SRCDIR([configure.ac])
AM_INIT_AUTOMAKE
AM_PROG_CC_C_O
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
                ARCH_SPECIFIC_CFLAGS=""
                ARCH_STRING="X86"
                ;;
        x86_64-*)
                AC_DEFINE(ARCH_X86_64, 1, [Architecture is x86-64])
                ARCH_SPECIFIC_CFLAGS="-DUSE_ADDR64"
                ARCH_STRING="X86-64"
                ;;
        ppc-* | powerpc-*)
                AC_DEFINE(ARCH_PPC, 1, [Architecture is PowerPC])
                ARCH_SPECIFIC_CFLAGS=""
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
# Package configuration switches.
#----------------------------------------------------------------------------
AC_ARG_ENABLE([multithreaded],
	[AC_HELP_STRING([--enable-multithreaded],
		[build BulletMultiThreaded (default NO)])],
	[disable_multithreaded=no], [disable_multithreaded=yes])
AC_MSG_CHECKING([BulletMultiThreaded])
AS_IF([test "$disable_multithreaded" = yes], [build_multithreaded=no], [build_multithreaded=yes])
AC_MSG_RESULT([$build_multithreaded])
AM_CONDITIONAL([CONDITIONAL_BUILD_MULTITHREADED], [test "$build_multithreaded" = yes])

AC_ARG_ENABLE([demos],
    [AS_HELP_STRING([--disable-demos],
	    [disable Bullet demos])],
    [],
    [enable_demos=yes])
AM_CONDITIONAL([CONDITIONAL_BUILD_DEMOS], [false])
if test "x$enable_demos" != xno; then
    AC_MSG_NOTICE([Building Bullet demos])
    AM_CONDITIONAL([CONDITIONAL_BUILD_DEMOS],[true])
fi



AC_ARG_ENABLE([debug],
    [AC_HELP_STRING([--enable-debug],
	[build with debugging information (default NO)])],
    [], [enable_debug=no])

AC_MSG_CHECKING([build mode])
AS_IF([test $enable_debug = yes], [build_mode=debug], [build_mode=optimize])
AC_MSG_RESULT([$build_mode])



CFLAGS="$ARCH_SPECIFIC_CFLAGS $CFLAGS"
CXXFLAGS="$ARCH_SPECIFIC_CFLAGS $CXXFLAGS $CFLAGS"
#----------------------------------------------------------------------------
# Emit generated files.
#----------------------------------------------------------------------------
AC_CONFIG_FILES([bullet.pc Makefile Demos/Makefile Demos/SoftDemo/Makefile Demos/AllBulletDemos/Makefile Demos/MultiThreadedDemo/Makefile  Demos/OpenGL/Makefile Demos/BasicDemo/Makefile Demos/CcdPhysicsDemo/Makefile Demos/VehicleDemo/Makefile Demos/TerrainDemo/Makefile src/Makefile Extras/Makefile])
AC_OUTPUT

AC_MSG_NOTICE([

Please type 'make' to build Bullet
])
