#! /bin/sh
set -x

if test ! -f configure.ac ; then
  echo "*** Please invoke this script from directory containing configure.ac."
  exit 1
fi

MACROFILE=aclocal.m4
MACRODIR=mk/autoconf

rm -f $MACROFILE

echo "Running aclocal"
aclocal

for i in $MACRODIR/*.m4 ; do
  cat $i >> $MACROFILE
done


echo "Running automake --add-missing"
automake --add-missing

echo "Running autoheader"
autoheader
rc=$?

if test $rc -eq 0; then
  echo "Running autoconf"
  autoconf
  rc=$?
fi

#rm -f $MACROFILE
exit $rc
