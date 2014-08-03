// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef EXAMPLES_TUMBLER_CALLBACK_H_
#define EXAMPLES_TUMBLER_CALLBACK_H_

#include <map>
#include <string>
#include <vector>

namespace tumbler {

class ScriptingBridge;

// Templates used to support method call-backs when a method or property is
// accessed from the browser code.

// Class suite used to publish a method name to Javascript.  Typical use is
// like this:
//     photo::MethodCallback<Calculator>* calculate_callback_;
//     calculate_callback_ =
//        new scripting::MethodCallback<Calculator>(this,
//                                                  &Calculator::Calculate);
//     bridge->AddMethodNamed("calculate", calculate_callback_);
//     ...
//     delete calculate_callback_;
//
// The caller must delete the callback.

// Methods get parameters as a dictionary that maps parameter names to values.
typedef std::map<std::string, std::string> MethodParameter;

// Pure virtual class used in STL containers.
class MethodCallbackExecutor {
 public:
  virtual ~MethodCallbackExecutor() {}
  virtual void Execute(
      const ScriptingBridge& bridge,
      const MethodParameter& parameters) = 0;
};

template <class T>
class MethodCallback : public MethodCallbackExecutor {
 public:
  typedef void (T::*Method)(
      const ScriptingBridge& bridge,
      const MethodParameter& parameters);

  MethodCallback(T* instance, Method method)
      : instance_(instance), method_(method) {}
  virtual ~MethodCallback() {}
  virtual void Execute(
      const ScriptingBridge& bridge,
      const MethodParameter& parameters) {
    // Use "this->" to force C++ to look inside our templatized base class; see
    // Effective C++, 3rd Ed, item 43, p210 for details.
    ((this->instance_)->*(this->method_))(bridge, parameters);
  }

 private:
  T* instance_;
  Method method_;
};

template <class T>
class ConstMethodCallback : public MethodCallbackExecutor {
 public:
  typedef void (T::*ConstMethod)(
      const ScriptingBridge& bridge,
      const MethodParameter& parameters) const;

  ConstMethodCallback(const T* instance, ConstMethod method)
      : instance_(instance), const_method_(method) {}
  virtual ~ConstMethodCallback() {}
  virtual void Execute(
      const ScriptingBridge& bridge,
      const MethodParameter& parameters) {
    // Use "this->" to force C++ to look inside our templatized base class; see
    // Effective C++, 3rd Ed, item 43, p210 for details.
    ((this->instance_)->*(this->const_method_))(bridge, parameters);
  }

 private:
  const T* instance_;
  ConstMethod const_method_;
};

}  // namespace tumbler

#endif  // EXAMPLES_TUMBLER_CALLBACK_H_

