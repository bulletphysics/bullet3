// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef EXAMPLES_TUMBLER_TUMBLER_H_
#define EXAMPLES_TUMBLER_TUMBLER_H_

#include <pthread.h>
#include <map>
#include <vector>

#include "cube.h"
#include "opengl_context.h"
#include "opengl_context_ptrs.h"
#include "scripting_bridge.h"
#include "ppapi/cpp/instance.h"

namespace tumbler {

class Tumbler : public pp::Instance {
 public:
  explicit Tumbler(PP_Instance instance);

  // The dtor makes the 3D context current before deleting the cube view, then
  // destroys the 3D context both in the module and in the browser.
  virtual ~Tumbler();

  // Called by the browser when the NaCl module is loaded and all ready to go.
  virtual bool Init(uint32_t argc, const char* argn[], const char* argv[]);

  // Called whenever the in-browser window changes size.
  virtual void DidChangeView(const pp::Rect& position, const pp::Rect& clip);

  // Called by the browser to handle the postMessage() call in Javascript.
  virtual void HandleMessage(const pp::Var& message);

  // Bind and publish the module's methods to JavaScript.
  void InitializeMethods(ScriptingBridge* bridge);

  // Set the camera orientation to the quaternion in |args[0]|.  |args| must
  // have length at least 1; the first element is expeted to be an Array
  // object containing 4 floating point number elements (the quaternion).
  // This method is bound to the JavaScript "setCameraOrientation" method and
  // is called like this:
  //     module.setCameraOrientation([0.0, 1.0, 0.0, 0.0]);
  void SetCameraOrientation(
    const tumbler::ScriptingBridge& bridge,
    const tumbler::MethodParameter& parameters);

  // Called to draw the contents of the module's browser area.
  void DrawSelf();

 private:
  // Browser connectivity and scripting support.
  ScriptingBridge scripting_bridge_;

  SharedOpenGLContext opengl_context_;
  // Wouldn't it be awesome if we had boost::scoped_ptr<>?
  Cube* cube_;
};

}  // namespace tumbler

#endif  // EXAMPLES_TUMBLER_TUMBLER_H_
