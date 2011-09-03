// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef EXAMPLES_TUMBLER_CUBE_H_
#define EXAMPLES_TUMBLER_CUBE_H_

#include <GLES2/gl2.h>
#include <vector>
#include "opengl_context.h"
#include "opengl_context_ptrs.h"

namespace tumbler {

// The Cube class provides a place to implement 3D rendering.  It has a
// frame that it occupies in a browser window.
class Cube {
 public:
  explicit Cube(SharedOpenGLContext opengl_context);
  ~Cube();

  // Called once when a new RenderContext is first bound to the view.  The
  // bound context is guaranteed to be current and valid before calling this
  // method.
  void PrepareOpenGL();

  // Called whenever the size of the browser view changes.  This method is
  // called at least once when the view is first made visible.  Clamps the
  // sizes to 1.
  void Resize(int width, int height);

  // Called every time the view need to be drawn.  The bound context is
  // guaranteed to be current and valid before this method is called.  The
  // visible portion of the context is flushed to the browser after this
  // method returns.
  void Draw();

  // Accessor for width and height.  To change these, call Resize.
  const int width() const {
    return width_;
  }

  const int height() const {
    return height_;
  }

  // Accessor/mutator for the camera orientation.
  void GetOrientation(std::vector<float>* orientation) const {
    if (!orientation)
      return;
    (*orientation)[0] = static_cast<float>(orientation_[0]);
    (*orientation)[1] = static_cast<float>(orientation_[1]);
    (*orientation)[2] = static_cast<float>(orientation_[2]);
    (*orientation)[3] = static_cast<float>(orientation_[3]);
  }
  void SetOrientation(const std::vector<float>& orientation) {
    orientation_[0] = static_cast<GLfloat>(orientation[0]);
    orientation_[1] = static_cast<GLfloat>(orientation[1]);
    orientation_[2] = static_cast<GLfloat>(orientation[2]);
    orientation_[3] = static_cast<GLfloat>(orientation[3]);
  }

 private:
  // Create the shaders used to draw the cube, and link them into a program.
  // Initializes |shader_progam_object_|, |position_loction_| and
  // |mvp_location_|.
  bool CreateShaders();

  // Generates a cube as a series of GL_TRIANGLE_STRIPs, and initializes
  // |index_count_| to the number of indices in the index list used as a VBO.
  // Creates the |vbo_ids_| required for the vertex and index data and uploads
  // the the VBO data.
  void CreateCube();

  // Build up the model-view transform from the eye and orienation properties.
  // Assumes that |model_view| is a 4x4 matrix.
  void ComputeModelViewTransform(GLfloat* model_view);

  SharedOpenGLContext opengl_context_;
  int width_;
  int height_;
  GLuint shader_program_object_;  // The compiled shaders.
  GLint position_location_;  // The position attribute location.
  GLint color_location_;  // The color attribute location.
  GLint mvp_location_;  // The Model-View-Projection composite matrix.
  GLuint cube_vbos_[3];
  GLfloat eye_[3];  // The eye point of the virtual camera.
  // The orientation of the virtual camera stored as a quaternion.  The
  // quaternion is laid out as {{x, y, z}, w}.
  GLfloat orientation_[4];
  GLfloat perspective_proj_[16];
  GLfloat mvp_matrix_[16];
};

}  // namespace tumbler

#endif  // EXAMPLES_TUMBLER_CUBE_H_
