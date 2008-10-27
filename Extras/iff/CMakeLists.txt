INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/Extras/iff
)

ADD_LIBRARY(LibIff
iff.h 
iffr.cpp
iffw.cpp
)
