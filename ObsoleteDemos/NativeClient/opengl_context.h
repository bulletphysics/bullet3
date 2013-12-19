// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef EXAMPLES_TUMBLER_OPENGL_CONTEXT_H_
#define EXAMPLES_TUMBLER_OPENGL_CONTEXT_H_

///
/// @file
/// OpenGLContext manages the OpenGL context in the browser that is associated
/// with a @a pp::Instance instance.
///

#include <pthread.h>

#include <algorithm>
#include <string>

#include "opengl_context_ptrs.h"
#include "ppapi/c/dev/ppb_opengles_dev.h"
#include "ppapi/cpp/dev/context_3d_dev.h"
#include "ppapi/cpp/dev/graphics_3d_client_dev.h"
#include "ppapi/cpp/dev/graphics_3d_dev.h"
#include "ppapi/cpp/dev/surface_3d_dev.h"
#include "ppapi/cpp/instance.h"

namespace tumbler {

/// OpenGLContext manages an OpenGL rendering context in the browser.
///
class OpenGLContext : public pp::Graphics3DClient_Dev {
 public:
  explicit OpenGLContext(pp::Instance* instance);

  /// Release all the in-browser resources used by this context, and make this
  /// context invalid.
  virtual ~OpenGLContext();

  /// The Graphics3DClient interfcace.
  virtual void Graphics3DContextLost() {
    assert(!"Unexpectedly lost graphics context");
  }

  /// Make @a this the current 3D context in @a instance.
  /// @param instance The instance of the NaCl module that will receive the
  ///                 the current 3D context.
  /// @return success.
  bool MakeContextCurrent(pp::Instance* instance);

  /// Flush the contents of this context to the browser's 3D device.
  void FlushContext();

  /// Make the underlying 3D device invalid, so that any subsequent rendering
  /// commands will have no effect.  The next call to MakeContextCurrent() will
  /// cause the underlying 3D device to get rebound and start receiving
  /// receiving rendering commands again.  Use InvalidateContext(), for
  /// example, when resizing the context's viewing area.
  void InvalidateContext(pp::Instance* instance);

  /// The OpenGL ES 2.0 interface.
  const struct PPB_OpenGLES2_Dev* gles2() const {
    return gles2_interface_;
  }

  /// The PP_Resource needed to make GLES2 calls through the Pepper interface.
  const PP_Resource gl_context() const {
    return context_.pp_resource();
  }

  /// Indicate whether a flush is pending.  This can only be called from the
  /// main thread; it is not thread safe.
  bool flush_pending() const {
    return flush_pending_;
  }
  void set_flush_pending(bool flag) {
    flush_pending_ = flag;
  }

 private:
  pp::Context3D_Dev context_;
  pp::Surface3D_Dev surface_;
  bool flush_pending_;

  const struct PPB_OpenGLES2_Dev* gles2_interface_;
};

}  // namespace tumbler

#endif  // EXAMPLES_TUMBLER_OPENGL_CONTEXT_H_

