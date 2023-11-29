INCLUDE_DIRECTORIES( ${BULLET_PHYSICS_SOURCE_DIR}/src
${BULLET_PHYSICS_SOURCE_DIR}/src
${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletFileLoader
${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BlenderSerialize
)

add_library(BlenderSerialize ${BULLET_BUILD_TYPE} dna249.cpp dna249-64bit.cpp bBlenderFile.cpp  bBlenderFile.h bMain.cpp bMain.h )
