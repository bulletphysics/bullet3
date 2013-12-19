// Copyright (c) 2011 The Native Client Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/**
 * @fileoverview  Implement a virtual trackball in the tumbler.Trackball
 * class.  This class maps 2D mouse events to 3D rotations by simulating a
 * trackball that you roll by dragging the mouse.  There are two principle
 * methods in the class: startAtPointInFrame which you use to begin a trackball
 * simulation and rollToPoint, which you use while dragging the mouse.  The
 * rollToPoint method returns a rotation expressed as a quaternion.
 */


// Requires tumbler.Application
// Requires tumbler.DragEvent
// Requires tumbler.Vector3

/**
 * Constructor for the Trackball object.  This class maps 2D mouse drag events
 * into 3D rotations by simulating a trackball.  The idea is to simulate
 * clicking on the trackball, and then rolling it as you drag the mouse.
 * The math behind the trackball is simple: start with a vector from the first
 * mouse-click on the ball to the center of the 3D view.  At the same time, set
 * the radius  of the ball to be the smaller dimension of the 3D view.  As you
 * drag the mouse around in the 3D view, a second vector is computed from the
 * surface of the ball to the center.  The axis of rotation is the cross
 * product of these two vectors, and the angle of rotation is the angle between
 * the two vectors.
 * @constructor
 */
tumbler.Trackball = function() {
  /**
   * The square of the trackball's radius.  The math never looks at the radius,
   * but looks at the radius squared.
   * @type {number}
   * @private
   */
  this.sqrRadius_ = 0;

  /**
   * The 3D vector representing the point on the trackball where the mouse
   * was clicked.  Default is pointing stright through the center of the ball.
   * @type {Object}
   * @private
   */
  this.rollStart_ = new tumbler.Vector3(0, 0, 1);

  /**
   * The 2D center of the frame that encloses the trackball.
   * @type {!Object}
   * @private
   */
  this.center_ = { x: 0, y: 0 };

  /**
   * Cached camera orientation.  When a drag START event happens this is set to
   * the current orientation in the calling view's plugin.  The default is the
   * identity quaternion.
   * @type {Array.<number>}
   * @private
   */
  this.cameraOrientation_ = [0, 0, 0, 1];
};

/**
 * Compute the dimensions of the virtual trackball to fit inside |frameSize|.
 * The radius of the trackball is set to be 1/2 of the smaller of the two frame
 * dimensions, the center point is at the midpoint of each side.
 * @param {!goog.math.Size} frameSize 2D-point representing the size of the
 *     element that encloses the virtual trackball.
 * @private
 */
tumbler.Trackball.prototype.initInFrame_ = function(frameSize) {
  // Compute the radius of the virtual trackball.  This is 1/2 of the smaller
  // of the frame's width and height.
  var halfFrameSize = 0.5 * Math.min(frameSize.width, frameSize.height);
  // Cache the square of the trackball's radius.
  this.sqrRadius_ = halfFrameSize * halfFrameSize;
  // Figure the center of the view.
  this.center_.x = frameSize.width * 0.5;
  this.center_.y = frameSize.height * 0.5;
};

/**
 * Method to convert (by translation) a 2D client point from a coordinate space
 * with origin in the lower-left corner of the client view to a space with
 * origin in the center of the client view.  Use this method before mapping the
 * 2D point to he 3D tackball point (see also the projectOnTrackball_() method).
 * Call the startAtPointInFrame before calling this method so that the
 * |center_| property is correctly initialized.
 * @param {!Object} clientPoint map this point to the coordinate space with
 *     origin in thecenter of the client view.
 * @return {Object} the converted point.
 * @private
 */
tumbler.Trackball.prototype.convertClientPoint_ = function(clientPoint) {
  var difference = { x: clientPoint.x - this.center_.x,
                     y: clientPoint.y - this.center_.y }
  return difference;
};

/**
 * Method to map a 2D point to a 3D point on the virtual trackball that was set
 * up using the startAtPointInFrame method.  If the point lies outside of the
 * radius of the virtual trackball, then the z-coordinate of the 3D point
 * is set to 0.
 * @param {!Object.<x, y>} point 2D-point in the coordinate space with origin
 *     in the center of the client view.
 * @return {tumbler.Vector3} the 3D point on the virtual trackball.
 * @private
 */
tumbler.Trackball.prototype.projectOnTrackball_ = function(point) {
  var sqrRadius2D = point.x * point.x + point.y * point.y;
  var zValue;
  if (sqrRadius2D > this.sqrRadius_) {
    // |point| lies outside the virtual trackball's sphere, so use a virtual
    // z-value of 0.  This is equivalent to clicking on the horizontal equator
    // of the trackball.
    zValue = 0;
  } else {
    // A sphere can be defined as: r^2 = x^2 + y^2 + z^2, so z =
    // sqrt(r^2 - (x^2 + y^2)).
    zValue = Math.sqrt(this.sqrRadius_ - sqrRadius2D);
  }
  var trackballPoint = new tumbler.Vector3(point.x, point.y, zValue);
  return trackballPoint;
};

/**
 * Method to start up the trackball.  The trackball works by pretending that a
 * ball encloses the 3D view.  You roll this pretend ball with the mouse.  For
 * example, if you click on the center of the ball and move the mouse straight
 * to the right, you roll the ball around its Y-axis.  This produces a Y-axis
 * rotation.  You can click on the "edge" of the ball and roll it around
 * in a circle to get a Z-axis rotation.
 * @param {!Object.<x, y>} startPoint 2D-point, usually the mouse-down
 *     point.
 * @param {!Object.<width, height>} frameSize 2D-point representing the size of
 *     the element that encloses the virtual trackball.
 */
tumbler.Trackball.prototype.startAtPointInFrame =
    function(startPoint, frameSize) {
  this.initInFrame_(frameSize);
  // Compute the starting vector from the surface of the ball to its center.
  this.rollStart_ = this.projectOnTrackball_(
      this.convertClientPoint_(startPoint));
};

/**
 * Method to roll the virtual trackball; call this in response to a mouseDrag
 * event.  Takes |dragPoint| and projects it from 2D mouse coordinates onto the
 * virtual track ball that was set up in startAtPointInFrame method.
 * Returns a quaternion that represents the rotation from |rollStart_| to
 * |rollEnd_|.
 * @param {!Object.<x, y>} dragPoint 2D-point representing the
 *     destination mouse point.
 * @return {Array.<number>} a quaternion that represents the rotation from
 *     the point wnere the mouse was clicked on the trackball to this point.
 *     The quaternion looks like this: [[v], cos(angle/2)], where [v] is the
 *     imaginary part of the quaternion and is computed as [x, y, z] *
 *     sin(angle/2).
 */
tumbler.Trackball.prototype.rollToPoint = function(dragPoint) {
  var rollTo = this.convertClientPoint_(dragPoint);
  if ((Math.abs(this.rollStart_.x - rollTo.x) <
               tumbler.Trackball.DOUBLE_EPSILON) &&
      (Math.abs(this.rollStart_.y, rollTo.y) <
               tumbler.Trackball.DOUBLE_EPSILON)) {
    // Not enough change in the vectors to roll the ball, return the identity
    // quaternion.
    return [0, 0, 0, 1];
  }

  // Compute the ending vector from the surface of the ball to its center.
  var rollEnd = this.projectOnTrackball_(rollTo);

  // Take the cross product of the two vectors. r = s X e
  var rollVector = this.rollStart_.cross(rollEnd);
  var invStartMag = 1.0 / this.rollStart_.magnitude();
  var invEndMag = 1.0 / rollEnd.magnitude();

  // cos(a) = (s . e) / (||s|| ||e||)
  var cosAng = this.rollStart_.dot(rollEnd) * invStartMag * invEndMag;
  // sin(a) = ||(s X e)|| / (||s|| ||e||)
  var sinAng = rollVector.magnitude() * invStartMag * invEndMag;
  // Build a quaternion that represents the rotation about |rollVector|.
  // Use atan2 for a better angle.  If you use only cos or sin, you only get
  // half the possible angles, and you can end up with rotations that flip
  // around near the poles.
  var rollHalfAngle = Math.atan2(sinAng, cosAng) * 0.5;
  rollVector.normalize();
  // The quaternion looks like this: [[v], cos(angle/2)], where [v] is the
  // imaginary part of the quaternion and is computed as [x, y, z] *
  // sin(angle/2).
  rollVector.scale(Math.sin(rollHalfAngle));
  var ballQuaternion = [rollVector.x,
                        rollVector.y,
                        rollVector.z,
                        Math.cos(rollHalfAngle)];
  return ballQuaternion;
};

/**
 * Handle the drag START event: grab the current camera orientation from the
 * sending view and set up the virtual trackball.
 * @param {!tumbler.Application} view The view controller that called this
 *     method.
 * @param {!tumbler.DragEvent} dragStartEvent The DRAG_START event that
 *     triggered this handler.
 */
tumbler.Trackball.prototype.handleStartDrag =
    function(controller, dragStartEvent) {
  // Cache the camera orientation.  The orientations from the trackball as it
  // rolls are concatenated to this orientation and pushed back into the
  // plugin on the other side of the JavaScript bridge.
  controller.setCameraOrientation(this.cameraOrientation_);
  // Invert the y-coordinate for the trackball computations.
  var frameSize = { width: controller.offsetWidth,
                    height: controller.offsetHeight };
  var flippedY = { x: dragStartEvent.clientX,
                   y: frameSize.height - dragStartEvent.clientY };
  this.startAtPointInFrame(flippedY, frameSize);
};

/**
 * Handle the drag DRAG event: concatenate the current orientation to the
 * cached orientation.  Send this final value through to the GSPlugin via the
 * setValueForKey() method.
 * @param {!tumbler.Application} view The view controller that called this
 *     method.
 * @param {!tumbler.DragEvent} dragEvent The DRAG event that triggered this
 *     handler.
 */
tumbler.Trackball.prototype.handleDrag =
    function(controller, dragEvent) {
  // Flip the y-coordinate so that the 2D origin is in the lower-left corner.
  var frameSize = { width: controller.offsetWidth,
                    height: controller.offsetHeight };
  var flippedY = { x: dragEvent.clientX,
                   y: frameSize.height - dragEvent.clientY };
  controller.setCameraOrientation(
      tumbler.multQuaternions(this.rollToPoint(flippedY),
                              this.cameraOrientation_));
};

/**
 * Handle the drag END event: get the final orientation and concatenate it to
 * the cached orientation.
 * @param {!tumbler.Application} view The view controller that called this
 *     method.
 * @param {!tumbler.DragEvent} dragEndEvent The DRAG_END event that triggered
 *     this handler.
 */
tumbler.Trackball.prototype.handleEndDrag =
    function(controller, dragEndEvent) {
  // Flip the y-coordinate so that the 2D origin is in the lower-left corner.
  var frameSize = { width: controller.offsetWidth,
                    height: controller.offsetHeight };
  var flippedY = { x: dragEndEvent.clientX,
                   y: frameSize.height - dragEndEvent.clientY };
  this.cameraOrientation_ = tumbler.multQuaternions(this.rollToPoint(flippedY),
                                                    this.cameraOrientation_);
  controller.setCameraOrientation(this.cameraOrientation_);
};

/**
 * A utility function to multiply two quaterions.  Returns the product q0 * q1.
 * This is effectively the same thing as concatenating the two rotations
 * represented in each quaternion together. Note that quaternion multiplication
 * is NOT commutative: q0 * q1 != q1 * q0.
 * @param {!Array.<number>} q0 A 4-element array representing the first
 *     quaternion.
 * @param {!Array.<number>} q1 A 4-element array representing the second
 *     quaternion.
 * @return {Array.<number>} A 4-element array representing the product q0 * q1.
 */
tumbler.multQuaternions = function(q0, q1) {
  // Return q0 * q1 (note the order).
  var qMult = [
      q0[3] * q1[0] + q0[0] * q1[3] + q0[1] * q1[2] - q0[2] * q1[1],
      q0[3] * q1[1] - q0[0] * q1[2] + q0[1] * q1[3] + q0[2] * q1[0],
      q0[3] * q1[2] + q0[0] * q1[1] - q0[1] * q1[0] + q0[2] * q1[3],
      q0[3] * q1[3] - q0[0] * q1[0] - q0[1] * q1[1] - q0[2] * q1[2]
  ];
  return qMult;
};

/**
 * Real numbers that are less than this distance apart are considered
 * equivalent.
 * TODO(dspringer): It seems as though there should be a const like this
 * in Closure somewhere (goog.math?).
 * @type {number}
 */
tumbler.Trackball.DOUBLE_EPSILON = 1.0e-16;
