// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "shader_util.h"

#include <stdlib.h>
#include <stdio.h>

namespace shader_util {

GLuint CreateShaderOfType(GLenum type, const char *shader_src) {
  GLuint shader;
  GLint compiled;

  // Create the shader object
  shader = glCreateShader(type);

  if (shader == 0)
    return 0;

  // Load and compile the shader source
  glShaderSource(shader, 1, &shader_src, NULL);
  glCompileShader(shader);

  // Check the compile status
  glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
  if (compiled == 0) {
    GLint info_len = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &info_len);
    if (info_len > 1) {
      char* info_log = reinterpret_cast<char*>(malloc(sizeof(char) * info_len));
      glGetShaderInfoLog(shader, info_len, NULL, info_log);
      // TODO(dspringer): We could really use a logging API.
      printf("Error compiling shader:\n%s\n", info_log);
      free(info_log);
    }
    glDeleteShader(shader);
    return 0;
  }

  return shader;
}

GLuint CreateProgramFromVertexAndFragmentShaders(
    const char *vertex_shader_src, const char *fragment_shader_src) {
  GLuint vertex_shader;
  GLuint fragment_shader;
  GLuint program_object;
  GLint linked;

  // Load the vertex/fragment shaders
  vertex_shader = CreateShaderOfType(GL_VERTEX_SHADER, vertex_shader_src);
  if (vertex_shader == 0)
    return 0;
  fragment_shader = CreateShaderOfType(GL_FRAGMENT_SHADER, fragment_shader_src);
  if (fragment_shader == 0) {
    glDeleteShader(vertex_shader);
    return 0;
  }

  // Create the program object and attach the shaders.
  program_object = glCreateProgram();
  if (program_object == 0)
    return 0;
  glAttachShader(program_object, vertex_shader);
  glAttachShader(program_object, fragment_shader);

  // Link the program
  glLinkProgram(program_object);

  // Check the link status
  glGetProgramiv(program_object, GL_LINK_STATUS, &linked);
  if (linked == 0) {
    GLint info_len = 0;
    glGetProgramiv(program_object, GL_INFO_LOG_LENGTH, &info_len);
    if (info_len > 1) {
      char* info_log = reinterpret_cast<char*>(malloc(info_len));
      glGetProgramInfoLog(program_object, info_len, NULL, info_log);
      // TODO(dspringer): We could really use a logging API.
      printf("Error linking program:\n%s\n", info_log);
      free(info_log);
    }
    glDeleteProgram(program_object);
    return 0;
  }

  // Delete these here because they are attached to the program object.
  glDeleteShader(vertex_shader);
  glDeleteShader(fragment_shader);

  return program_object;
}

}  // namespace shader_util
