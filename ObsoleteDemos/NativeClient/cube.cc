// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "cube.h"

#include <algorithm>

#include "shader_util.h"
#include "transforms.h"

namespace tumbler {

static const size_t kVertexCount = 24;
static const int kIndexCount = 36;

Cube::Cube(SharedOpenGLContext opengl_context)
    : opengl_context_(opengl_context),
      width_(1),
      height_(1) {
  eye_[0] = eye_[1] = 0.0f;
  eye_[2] = 2.0f;
  orientation_[0] = 0.0f;
  orientation_[1] = 0.0f;
  orientation_[2] = 0.0f;
  orientation_[3] = 1.0f;
}

Cube::~Cube() {
  glDeleteBuffers(3, cube_vbos_);
  glDeleteProgram(shader_program_object_);
}

void Cube::PrepareOpenGL() {
  CreateShaders();
  CreateCube();
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glEnable(GL_DEPTH_TEST);
}

void Cube::Resize(int width, int height) {
  width_ = std::max(width, 1);
  height_ = std::max(height, 1);
  // Set the viewport
  glViewport(0, 0, width_, height_);
  // Compute the perspective projection matrix with a 60 degree FOV.
  GLfloat aspect = static_cast<GLfloat>(width_) / static_cast<GLfloat>(height_);
  transform_4x4::LoadIdentity(perspective_proj_);
  transform_4x4::Perspective(perspective_proj_, 60.0f, aspect, 1.0f, 20.0f);
}

void Cube::Draw() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Compute a new model-view matrix, then use that to make the composite
  // model-view-projection matrix: MVP = MV . P.
  GLfloat model_view[16];
  ComputeModelViewTransform(model_view);
  transform_4x4::Multiply(mvp_matrix_, model_view, perspective_proj_);

  glBindBuffer(GL_ARRAY_BUFFER, cube_vbos_[0]);
  glUseProgram(shader_program_object_);
  glEnableVertexAttribArray(position_location_);
  glVertexAttribPointer(position_location_,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        3 * sizeof(GLfloat),
                        NULL);
  glEnableVertexAttribArray(color_location_);
  glBindBuffer(GL_ARRAY_BUFFER, cube_vbos_[1]);
  glVertexAttribPointer(color_location_,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        3 * sizeof(GLfloat),
                        NULL);
  glUniformMatrix4fv(mvp_location_, 1, GL_FALSE, mvp_matrix_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cube_vbos_[2]);
  glDrawElements(GL_TRIANGLES, kIndexCount, GL_UNSIGNED_SHORT, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

bool Cube::CreateShaders() {
  const char vertex_shader_src[] =
    "uniform mat4 u_mvpMatrix;                   \n"
    "attribute vec4 a_position;                  \n"
    "attribute vec3 a_color;                     \n"
    "varying lowp vec4 v_color;                  \n"
    "void main()                                 \n"
    "{                                           \n"
    "   v_color.xyz = a_color;                   \n"
    "   v_color.w = 1.0;                         \n"
    "   gl_Position = u_mvpMatrix * a_position;  \n"
    "}                                           \n";

  const char fragment_shader_src[] =
    "varying lowp vec4 v_color;                   \n"
    "void main()                                  \n"
    "{                                            \n"
    "  gl_FragColor = v_color;                    \n"
    "}                                            \n";

  // Load the shaders and get a linked program object
  shader_program_object_ =
      shader_util::CreateProgramFromVertexAndFragmentShaders(
      vertex_shader_src, fragment_shader_src);
  if (shader_program_object_ == 0)
    return false;
  position_location_ = glGetAttribLocation(shader_program_object_,
                                           "a_position");
  color_location_ = glGetAttribLocation(shader_program_object_, "a_color");
  mvp_location_ = glGetUniformLocation(shader_program_object_, "u_mvpMatrix");
  return true;
}

void Cube::CreateCube() {
  static const GLfloat cube_vertices[] = {
    // Vertex coordinates interleaved with color values
    // Bottom
    -0.5f, -0.5f, -0.5f,
    -0.5f, -0.5f, 0.5f,
    0.5f, -0.5f, 0.5f,
    0.5f, -0.5f, -0.5f,
    // Top
    -0.5f, 0.5f, -0.5f,
    -0.5f, 0.5f, 0.5f,
    0.5f, 0.5f, 0.5f,
    0.5f, 0.5f, -0.5f,
    // Back
    -0.5f, -0.5f, -0.5f,
    -0.5f, 0.5f, -0.5f,
    0.5f, 0.5f, -0.5f,
    0.5f, -0.5f, -0.5f,
    // Front
    -0.5f, -0.5f, 0.5f,
    -0.5f, 0.5f, 0.5f,
    0.5f, 0.5f, 0.5f,
    0.5f, -0.5f, 0.5f,
    // Left
    -0.5f, -0.5f, -0.5f,
    -0.5f, -0.5f, 0.5f,
    -0.5f, 0.5f, 0.5f,
    -0.5f, 0.5f, -0.5f,
    // Right
    0.5f, -0.5f, -0.5f,
    0.5f, -0.5f, 0.5f,
    0.5f, 0.5f, 0.5f,
    0.5f, 0.5f, -0.5f
  };

  static const GLfloat cube_colors[] = {
    // Vertex coordinates interleaved with color values
    // Bottom
    1.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    // Top
    0.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    // Back
    0.0, 0.0, 1.0,
    0.0, 0.0, 1.0,
    0.0, 0.0, 1.0,
    0.0, 0.0, 1.0,
    // Front
    1.0, 0.0, 1.0,
    1.0, 0.0, 1.0,
    1.0, 0.0, 1.0,
    1.0, 0.0, 1.0,
    // Left
    1.0, 1.0, 0.0,
    1.0, 1.0, 0.0,
    1.0, 1.0, 0.0,
    1.0, 1.0, 0.0,
    // Right
    0.0, 1.0, 1.0,
    0.0, 1.0, 1.0,
    0.0, 1.0, 1.0,
    0.0, 1.0, 1.0
  };

  static const GLushort cube_indices[] = {
    // Bottom
    0, 2, 1,
    0, 3, 2,
    // Top
    4, 5, 6,
    4, 6, 7,
    // Back
    8, 9, 10,
    8, 10, 11,
    // Front
    12, 15, 14,
    12, 14, 13,
    // Left
    16, 17, 18,
    16, 18, 19,
    // Right
    20, 23, 22,
    20, 22, 21
  };

  // Generate the VBOs and upload them to the graphics context.
  glGenBuffers(3, cube_vbos_);
  glBindBuffer(GL_ARRAY_BUFFER, cube_vbos_[0]);
  glBufferData(GL_ARRAY_BUFFER,
               kVertexCount * sizeof(GLfloat) * 3,
               cube_vertices,
               GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, cube_vbos_[1]);
  glBufferData(GL_ARRAY_BUFFER,
               kVertexCount * sizeof(GLfloat) * 3,
               cube_colors,
               GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cube_vbos_[2]);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               kIndexCount * sizeof(GL_UNSIGNED_SHORT),
               cube_indices,
               GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Cube::ComputeModelViewTransform(GLfloat* model_view) {
  // This method takes into account the possiblity that |orientation_|
  // might not be normalized.
  double sqrx = orientation_[0] * orientation_[0];
  double sqry = orientation_[1] * orientation_[1];
  double sqrz = orientation_[2] * orientation_[2];
  double sqrw = orientation_[3] * orientation_[3];
  double sqrLength = 1.0 / (sqrx + sqry + sqrz + sqrw);

  transform_4x4::LoadIdentity(model_view);
  model_view[0] = (sqrx - sqry - sqrz + sqrw) * sqrLength;
  model_view[5] = (-sqrx + sqry - sqrz + sqrw) * sqrLength;
  model_view[10] = (-sqrx - sqry + sqrz + sqrw) * sqrLength;

  double temp1 = orientation_[0] * orientation_[1];
  double temp2 = orientation_[2] * orientation_[3];
  model_view[1] = 2.0 * (temp1 + temp2) * sqrLength;
  model_view[4] = 2.0 * (temp1 - temp2) * sqrLength;

  temp1 = orientation_[0] * orientation_[2];
  temp2 = orientation_[1] * orientation_[3];
  model_view[2] = 2.0 * (temp1 - temp2) * sqrLength;
  model_view[8] = 2.0 * (temp1 + temp2) * sqrLength;
  temp1 = orientation_[1] * orientation_[2];
  temp2 = orientation_[0] * orientation_[3];
  model_view[6] = 2.0 * (temp1 + temp2) * sqrLength;
  model_view[9] = 2.0 * (temp1 - temp2) * sqrLength;
  model_view[3] = 0.0;
  model_view[7] = 0.0;
  model_view[11] = 0.0;

  // Concatenate the translation to the eye point.
  model_view[12] = -eye_[0];
  model_view[13] = -eye_[1];
  model_view[14] = -eye_[2];
  model_view[15] = 1.0;
}

}  // namespace tumbler
