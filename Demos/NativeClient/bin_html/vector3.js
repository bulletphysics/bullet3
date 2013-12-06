// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/**
 * @fileoverview  A 3D vector class.  Proviudes some utility functions on
 * 3-dimentional vectors.
 */

// Requires tumbler

/**
 * Constructor for the Vector3 object.  This class contains a 3-tuple that
 * represents a vector in 3D space.
 * @param {?number} opt_x The x-coordinate for this vector.  If null or
 *     undefined, the x-coordinate value is set to 0.
 * @param {?number} opt_y The y-coordinate for this vector.  If null or
 *     undefined, the y-coordinate value is set to 0.
 * @param {?number} opt_z The z-coordinate for this vector.  If null or
 *     undefined, the z-coordinate value is set to 0.
 * @constructor
 */
tumbler.Vector3 = function(opt_x, opt_y, opt_z) {
  /**
   * The vector's 3-tuple.
   * @type {number}
   */
  this.x = opt_x || 0;
  this.y = opt_y || 0;
  this.z = opt_z || 0;
}

/**
 * Method to return the magnitude of a Vector3.
 * @return {number} the magnitude of the vector.
 */
tumbler.Vector3.prototype.magnitude = function() {
  return Math.sqrt(this.dot(this));
}

/**
 * Normalize the vector in-place.
 * @return {number} the magnitude of the vector.
 */
tumbler.Vector3.prototype.normalize = function() {
  var mag = this.magnitude();
  if (mag < tumbler.Vector3.DOUBLE_EPSILON)
    return 0.0;  // |this| is equivalent to the 0-vector, don't normalize.
  this.scale(1.0 / mag);
  return mag;
}

/**
 * Scale the vector in-place by |s|.
 * @param {!number} s The scale factor.
 */
tumbler.Vector3.prototype.scale = function(s) {
  this.x *= s;
  this.y *= s;
  this.z *= s;
}

/**
 * Compute the dot product: |this| . v.
 * @param {!tumbler.Vector3} v The vector to dot.
 * @return {number} the result of |this| . v.
 */
tumbler.Vector3.prototype.dot = function(v) {
  return this.x * v.x + this.y * v.y + this.z * v.z;
}

/**
 * Compute the cross product: |this| X v.
 * @param {!tumbler.Vector3} v The vector to cross with.
 * @return {tumbler.Vector3} the result of |this| X v.
 */
tumbler.Vector3.prototype.cross = function(v) {
  var vCross = new tumbler.Vector3(this.y * v.z - this.z * v.y,
                                   this.z * v.x - this.x * v.z,
                                   this.x * v.y - this.y * v.x);
  return vCross;
}

/**
 * Real numbers that are less than this distance apart are considered
 * equivalent.
 * TODO(dspringer): It seems as though there should be a const like this
 * in generally available somewhere.
 * @type {number}
 */
tumbler.Vector3.DOUBLE_EPSILON = 1.0e-16;
