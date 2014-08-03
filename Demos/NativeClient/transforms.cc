// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "transforms.h"

#include <math.h>
#include <string.h>
#include <GLES2/gl2.h>

namespace transform_4x4 {

static const GLfloat kPI = 3.1415926535897932384626433832795f;

void Translate(GLfloat* m, GLfloat tx, GLfloat ty, GLfloat tz) {
  m[12] += (m[0] * tx + m[4] * ty + m[8] * tz);
  m[13] += (m[1] * tx + m[5] * ty + m[9] * tz);
  m[14] += (m[2] * tx + m[6] * ty + m[10] * tz);
  m[15] += (m[3] * tx + m[7] * ty + m[11] * tz);
}

void Frustum(GLfloat* m,
             GLfloat left,
             GLfloat right,
             GLfloat bottom,
             GLfloat top,
             GLfloat near_z,
             GLfloat far_z) {
  GLfloat delta_x = right - left;
  GLfloat delta_y = top - bottom;
  GLfloat delta_z = far_z - near_z;
  GLfloat frustum[16];

  if ((near_z <= 0.0f) || (far_z <= 0.0f) ||
     (delta_x <= 0.0f) || (delta_y <= 0.0f) || (delta_z <= 0.0f))
     return;

  frustum[0] = 2.0f * near_z / delta_x;
  frustum[1] = frustum[2] = frustum[3] = 0.0f;

  frustum[5] = 2.0f * near_z / delta_y;
  frustum[4] = frustum[6] = frustum[7] = 0.0f;

  frustum[8] = (right + left) / delta_x;
  frustum[9] = (top + bottom) / delta_y;
  frustum[10] = -(near_z + far_z) / delta_z;
  frustum[11] = -1.0f;

  frustum[14] = -2.0f * near_z * far_z / delta_z;
  frustum[12] = frustum[13] = frustum[15] = 0.0f;

  transform_4x4::Multiply(m, frustum, m);
}


void Perspective(GLfloat* m,
                 GLfloat fovy,
                 GLfloat aspect,
                 GLfloat near_z,
                 GLfloat far_z) {
  GLfloat frustum_w, frustum_h;

  frustum_h = tanf((fovy * 0.5f) / 180.0f * kPI) * near_z;
  frustum_w = frustum_h * aspect;
  transform_4x4::Frustum(m, -frustum_w, frustum_w, -frustum_h, frustum_h,
                         near_z, far_z);
}

void Multiply(GLfloat *m, GLfloat *a, GLfloat* b) {
  GLfloat tmp[16];
  // tmp = a . b
  GLfloat a0, a1, a2, a3;
  a0 = a[0];
  a1 = a[1];
  a2 = a[2];
  a3 = a[3];
  tmp[0] = a0 * b[0] + a1 * b[4] + a2 * b[8] + a3 * b[12];
  tmp[1] = a0 * b[1] + a1 * b[5] + a2 * b[9] + a3 * b[13];
  tmp[2] = a0 * b[2] + a1 * b[6] + a2 * b[10] + a3 * b[14];
  tmp[3] = a0 * b[3] + a1 * b[7] + a2 * b[11] + a3 * b[15];

  a0 = a[4];
  a1 = a[5];
  a2 = a[6];
  a3 = a[7];
  tmp[4] = a0 * b[0] + a1 * b[4] + a2 * b[8] + a3 * b[12];
  tmp[5] = a0 * b[1] + a1 * b[5] + a2 * b[9] + a3 * b[13];
  tmp[6] = a0 * b[2] + a1 * b[6] + a2 * b[10] + a3 * b[14];
  tmp[7] = a0 * b[3] + a1 * b[7] + a2 * b[11] + a3 * b[15];

  a0 = a[8];
  a1 = a[9];
  a2 = a[10];
  a3 = a[11];
  tmp[8] = a0 * b[0] + a1 * b[4] + a2 * b[8] + a3 * b[12];
  tmp[9] = a0 * b[1] + a1 * b[5] + a2 * b[9] + a3 * b[13];
  tmp[10] = a0 * b[2] + a1 * b[6] + a2 * b[10] + a3 * b[14];
  tmp[11] = a0 * b[3] + a1 * b[7] + a2 * b[11] + a3 * b[15];

  a0 = a[12];
  a1 = a[13];
  a2 = a[14];
  a3 = a[15];
  tmp[12] = a0 * b[0] + a1 * b[4] + a2 * b[8] + a3 * b[12];
  tmp[13] = a0 * b[1] + a1 * b[5] + a2 * b[9] + a3 * b[13];
  tmp[14] = a0 * b[2] + a1 * b[6] + a2 * b[10] + a3 * b[14];
  tmp[15] = a0 * b[3] + a1 * b[7] + a2 * b[11] + a3 * b[15];
  memcpy(m, tmp, sizeof(GLfloat) * 4 * 4);
}

void LoadIdentity(GLfloat* m) {
  memset(m, 0, sizeof(GLfloat) * 4 * 4);
  m[0] = m[5] = m[10] = m[15] = 1.0f;
}

}  // namespace transform_4x4
