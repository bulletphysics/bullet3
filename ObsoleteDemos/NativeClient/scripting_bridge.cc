// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "scripting_bridge.h"

namespace {
const char* const kWhiteSpaceCharacters = " \t";

// Helper function to pull out the next token in |token_string|.  A token is
// delimited by whitespace.  Scanning begins at |*pos|, if pos goes beyond the
// end of |token_string|, it is set to std::string::npos and an empty string
// is returned.  On return, |*pos| will point to the beginning of the next
// token. |pos| must not be NULL.
const std::string ScanToken(const std::string& token_string, size_t* pos) {
  std::string token;
  if (*pos == std::string::npos) {
    return token;
  }
  size_t token_start_pos = token_string.find_first_not_of(kWhiteSpaceCharacters,
                                                          *pos);
  size_t token_end_pos = token_string.find_first_of(kWhiteSpaceCharacters,
                                                    token_start_pos);
  if (token_start_pos != std::string::npos) {
    token = token_string.substr(token_start_pos, token_end_pos);
  }
  *pos = token_end_pos;
  return token;
}

// Take a string of the form 'name:value' and split it into two strings, one
// containing 'name' and the other 'value'.  If the ':' separator is missing,
// or is the last character in |parameter|, |parameter| is copied to
// |param_name|, |param_value| is left unchanged and false is returned.
bool ParseParameter(const std::string& parameter,
                    std::string* param_name,
                    std::string* param_value) {
  bool success = false;
  size_t sep_pos = parameter.find_first_of(':');
  if (sep_pos != std::string::npos) {
    *param_name = parameter.substr(0, sep_pos);
    if (sep_pos < parameter.length() - 1) {
      *param_value = parameter.substr(sep_pos + 1);
      success = true;
    } else {
      success = false;
    }
  } else {
    *param_name = parameter;
    success = false;
  }
  return success;
}
}  // namespace

namespace tumbler {

bool ScriptingBridge::AddMethodNamed(const std::string& method_name,
                                     SharedMethodCallbackExecutor method) {
  if (method_name.size() == 0 || method == NULL)
    return false;
  method_dictionary_.insert(
      std::pair<std::string, SharedMethodCallbackExecutor>(method_name,
                                                           method));
  return true;
}

bool ScriptingBridge::InvokeMethod(const std::string& method) {
  size_t current_pos = 0;
  const std::string method_name = ScanToken(method, &current_pos);
  MethodDictionary::iterator method_iter;
  method_iter = method_dictionary_.find(method_name);
  if (method_iter != method_dictionary_.end()) {
    // Pull out the method parameters and build a dictionary that maps
    // parameter names to values.
    std::map<std::string, std::string> param_dict;
    while (current_pos != std::string::npos) {
      const std::string parameter = ScanToken(method, &current_pos);
      if (parameter.length()) {
        std::string param_name;
        std::string param_value;
        if (ParseParameter(parameter, &param_name, &param_value)) {
          // Note that duplicate parameter names will override each other.  The
          // last one in the method string will be used.
          param_dict[param_name] = param_value;
        }
      }
    }
    (*method_iter->second).Execute(*this, param_dict);
    return true;
  }
  return false;
}

}  // namespace tumbler
