#! /bin/sh

if [ "$USER" = "root" ]; then
	echo "*** You cannot do this as "$USER" please use a normal user account."
	exit 1
fi
if test ! -f configure.ac ; then
	echo "*** Please invoke this script from directory containing configure.ac."
	exit 1
fi

aclocal -I mk/autoconf
rc=$?

if test $rc -eq 0; then
	libtoolize --force --automake --copy
	rc=$?
fi

if test $rc -eq 0; then
	automake --add-missing --copy
	rc=$?
fi

if test $rc -eq 0; then
	autoheader
	rc=$?
fi

if test $rc -eq 0; then
	autoconf
	rc=$?
fi

echo "autogen.sh complete"
exit $rc
