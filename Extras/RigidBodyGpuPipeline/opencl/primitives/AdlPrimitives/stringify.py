#!/usr/bin/env python
import sys
import os
import shutil

arg = sys.argv[1]
fh = open(arg)
	
print 'static const char* '+sys.argv[2]+'= \\'
for line in fh.readlines():
	a = line.strip('\n')
	print '"'+a+'\\n"'
print ';'
