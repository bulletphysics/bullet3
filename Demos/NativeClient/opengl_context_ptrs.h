// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef EXAMPLES_TUMBLER_OPENGL_CONTEXT_PTRS_H_
#define EXAMPLES_TUMBLER_OPENGL_CONTEXT_PTRS_H_

// A convenience wrapper for a shared OpenGLContext pointer type.  As other
// smart pointer types are needed, add them here.

#include <tr1/memory>

namespace tumbler {

class OpenGLContext;

typedef std::tr1::shared_ptr<OpenGLContext> SharedOpenGLContext;

}  // namespace tumbler

#endif  // EXAMPLES_TUMBLER_OPENGL_CONTEXT_PTRS_H_

