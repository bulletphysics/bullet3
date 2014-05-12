// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/**
 * @fileoverview  This class implements an extension to Function object that
 * lets you bind a scope for |this| to a function.
 */

/**
 * Bind a scope to a function.  Used to bind an object to |this| for event
 * handlers.
 * @param {!Object} scope The scope in which the function executes.  |scope|
 *     becomes |this| during function execution.
 * @return {function} the bound version of the original function.
 */
Function.prototype.bind = function(scope) {
  var boundContext = this;
  return function() {
    return boundContext.apply(scope, arguments);
  }
}
