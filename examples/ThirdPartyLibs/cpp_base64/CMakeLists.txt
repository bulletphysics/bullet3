
add_library(cpp_base64 ${BULLET_BUILD_TYPE} base64.cpp)

target_include_directories(cpp_base64 PUBLIC include PRIVATE include/cpp_base64)


