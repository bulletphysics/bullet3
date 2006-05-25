#!/usr/bin/make

# FCollada makefile for PS3

SUBDIRS=LibXML FMath FUtils FCDocument

all:
	for i in $(SUBDIRS); do make -C $$i all install; done

clean:
	for i in $(SUBDIRS); do make -C $$i clean; done

