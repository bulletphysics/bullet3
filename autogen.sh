#! /bin/sh

if [ "$USER" = "root" ]; then
	echo "*** You cannot do this as "$USER" please use a normal user account."
	exit 1
fi
if test ! -f configure.ac ; then
	echo "*** Please invoke this script from directory containing configure.ac."
	exit 1
fi

#MACROFILE=aclocal.m4
#MACRODIR=mk/autoconf
#rm -f $MACROFILE

aclocal -I mk/autoconf
rc=$?

#for i in $MACRODIR/*.m4 ; do
#  cat $i >> $MACROFILE
#done

if test $rc -eq 0; then
	automake --add-missing
	rc=$?
fi

if test $rc -eq 0; then
	libtoolize --force
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

exit $rc
