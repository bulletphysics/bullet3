// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef EXAMPLES_TUMBLER_TRANSFORMS_H_
#define EXAMPLES_TUMBLER_TRANSFORMS_H_

#include <GLES2/gl2.h>

// A very simple set of 4x4 matrix routines.  In all these routines, the input
// matrix is assumed to be a 4x4 of GLfloats.

namespace transform_4x4 {

// Pre-multply |m| with a projection transformation 4x4 matrix from a
// truncated pyramid viewing frustum.
void Frustum(GLfloat* m,
             GLfloat left,
             GLfloat right,
             GLfloat bottom,
             GLfloat top,
             GLfloat near_z,
             GLfloat far_z);

// Replace |m| with the 4x4 identity matrix.
void LoadIdentity(GLfloat* m);

// |m| <- |a| . |b|.  |m| can point at the same memory as either |a| or |b|.
void Multiply(GLfloat *m, GLfloat *a, GLfloat* b);

// Pre-multiply |m| with a single-point perspective matrix based on the viewing
// frustum whose view angle is |fovy|.
void Perspective(GLfloat* m,
                 GLfloat fovy,
                 GLfloat aspect,
                 GLfloat near_z,
                 GLfloat far_z);

// Pre-multiply |m| with a matrix that represents a translation by |tx|, |ty|,
// |tz|.
void Translate(GLfloat* m, GLfloat tx, GLfloat ty, GLfloat tz);
}  // namespace transform_4x4

#endif  // EXAMPLES_TUMBLER_TRANSFORMS_H_

