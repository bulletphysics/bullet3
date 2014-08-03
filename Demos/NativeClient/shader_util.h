// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Some simple helper functions that load shaders and create program objects.

#ifndef EXAMPLES_TUMBLER_SHADER_UTIL_H_
#define EXAMPLES_TUMBLER_SHADER_UTIL_H_

#include <GLES2/gl2.h>

namespace shader_util {

// Load and compile a shader.  |type| can be one of GL_VERTEX_SHADER or
// GL_FRAGMENT_SHADER.  Returns a non-0 value representing the compiled
// shader on success, 0 on failure.  The caller is responsible for deleting
// the returned shader using glDeleteShader().
GLuint CreateShaderOfType(GLenum type, const char *shader_src);

// Load and compile the vertex and fragment shaders, then link these together
// into a complete program.  Returns a non-0 value representing the program on,
// success or 0 on failure.  The caller is responsible for deleting the
// returned program using glDeleteProgram().
GLuint CreateProgramFromVertexAndFragmentShaders(
    const char *vertex_shader_src, const char *fragment_shader_src);

}  // namespace shader_util

#endif  // EXAMPLES_TUMBLER_SHADER_UTIL_H_
