# Note by Nikolaus Demmel 28.03.2014: My contributions are licensend under the
# same as CMake (BSD). My adaptations are in part based
# https://github.com/qgis/QGIS/tree/master/cmake which has the following
# copyright note:

# FindLibPython.py
# Copyright (c) 2007, Simon Edwards <simon@simonzone.com>
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

import sys
import distutils.sysconfig

print("exec_prefix:%s" % sys.exec_prefix)
print("major_version:%s" % str(sys.version_info[0]))
print("minor_version:%s" % str(sys.version_info[1]))
print("patch_version:%s" % str(sys.version_info[2]))
print("short_version:%s" % '.'.join(map(lambda x: str(x), sys.version_info[0:2])))
print("long_version:%s" % '.'.join(map(lambda x: str(x), sys.version_info[0:3])))
print("py_inc_dir:%s" % distutils.sysconfig.get_python_inc())
print("site_packages_dir:%s" % distutils.sysconfig.get_python_lib(plat_specific=1))
for e in distutils.sysconfig.get_config_vars('LIBDIR'):
  if e != None:
    print("py_lib_dir:%s" % e)
    break
