INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/Extras/GIMPACT/include
${BULLET_PHYSICS_SOURCE_DIR}/src
${BULLET_PHYSICS_SOURCE_DIR}/Extras/GIMPACTUtils
${BULLET_PHYSICS_SOURCE_DIR}/Extras/ConvexDecomposition
)

ADD_LIBRARY(LibGIMPACTUtils
btGImpactConvexDecompositionShape.cpp	btGImpactConvexDecompositionShape.h
)
