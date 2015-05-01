
INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/Glew
)

ADD_DEFINITIONS( -DGLEW_STATIC -DGWEN_COMPILE_STATIC -D_HAS_EXCEPTIONS=0 -D_STATIC_CPPLIB )

FILE(GLOB gwen_SRCS "*.cpp" "Controls/*.cpp" "Controls/Dialog/*.cpp" "Controls/Dialogs/*.cpp" "Controls/Layout/*.cpp" "Controls/Property/*.cpp" "Input/*.cpp" "Platforms/*.cpp" "Renderers/*.cpp" "Skins/*.cpp")
FILE(GLOB gwen_HDRS "*.h" "Controls/*.h" "Controls/Dialog/*.h" "Controls/Dialogs/*.h" "Controls/Layout/*.h" "Controls/Property/*.h" "Input/*.h" "Platforms/*.h" "Renderers/*.h" "Skins/*.h")

ADD_LIBRARY(gwen ${gwen_SRCS} ${gwen_HDRS})
if (BUILD_SHARED_LIBS)
  target_link_libraries(gwen ${OPENGL_gl_LIBRARY})
endif()
