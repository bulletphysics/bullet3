#!/usr/bin/make
# FMath Makefile for PS3

include ../../MakeDefs

LIBNAME = libfmath

prefix=..

INCPATH=$(prefix)/include
LIBPATH=$(prefix)/lib

PPU_INCDIRS=-I$(INCPATH) -I..
PPU_OPTIMIZE_LV= -O3 -funroll-loops

PPU_LIB_TARGET=$(LIBNAME).a
PPU_SRCS = \
	FMColor.cpp \
	FMInterpolation.cpp \
	FMMatrix33.cpp \
	FMMatrix44.cpp \
	FMQuaternion.cpp \
	FMVector3.cpp \
	StdAfx.cpp

include ../../MakeRules
