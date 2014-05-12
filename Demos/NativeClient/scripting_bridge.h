// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef EXAMPLES_TUMBLER_SCRIPTING_BRIDGE_H_
#define EXAMPLES_TUMBLER_SCRIPTING_BRIDGE_H_

#include <map>
#include <string>
#include <tr1/memory>
#include <vector>

#include "callback.h"
#include "ppapi/cpp/var.h"

namespace tumbler {

class MethodCallbackExecutor;

// This class handles the interface between the browser and the NaCl module.
// There is a single point of entry from the browser: postMessage().  The
// string passed to postMessage() has this format:
//     'function_name arg_name0:arg_0 arg_name1:arg1 ...'
// The arguments have undetermined type; they are placed in a map of argument
// names and values.  Values are all strings, it is up to the target code to
// do any type coercion.
// Methods called by the scripting bridge must have a signature like this:
//     void Method(const ScriptingBridge& bridge,
//                 const ParameterDictionary&);
class ScriptingBridge {
 public:
  // Shared pointer type used in the method map.
  typedef std::tr1::shared_ptr<MethodCallbackExecutor>
      SharedMethodCallbackExecutor;

  virtual ~ScriptingBridge() {}

  // Causes |method_name| to be published as a method that can be called via
  // postMessage() from the browser.  Associates this method with |method|.
  bool AddMethodNamed(const std::string& method_name,
                      SharedMethodCallbackExecutor method);

  bool InvokeMethod(const std::string& method);

 private:
  typedef std::map<std::string, SharedMethodCallbackExecutor> MethodDictionary;

  MethodDictionary method_dictionary_;
};

}  // namespace tumbler
#endif  // EXAMPLES_TUMBLER_SCRIPTING_BRIDGE_H_
