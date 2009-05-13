#!/bin/bash

g++ -O3 -L./common -I./common -L./particles -I./particles -lGL -lglut common/matrix.cpp common/mdebug.cpp common/merror.cpp common/vector.cpp common/mtime.cpp particles/particle.cpp particles/particle_system.cpp GLee.c main.cpp 
g++ -g -L./common -I./common -L./particles -I./particles -lGL -lglut common/matrix.cpp common/mdebug.cpp common/merror.cpp common/vector.cpp common/mtime.cpp particles/particle.cpp particles/particle_system.cpp GLee.c main.cpp -o sph_sim.debug

