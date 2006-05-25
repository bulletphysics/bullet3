#! /bin/sh

if test ! -f configure.ac ; then
  echo "*** Please invoke this script from directory containing configure.ac."
  exit 1
fi

MACROFILE=aclocal.m4
MACRODIR=mk/autoconf

rm -f $MACROFILE
for i in $MACRODIR/*.m4 ; do
  cat $i >> $MACROFILE
done

autoheader
rc=$?

if test $rc -eq 0; then
  autoconf
  rc=$?
fi

rm -f $MACROFILE
exit $rc
