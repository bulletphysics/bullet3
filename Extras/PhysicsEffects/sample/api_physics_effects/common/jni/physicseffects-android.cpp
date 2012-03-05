/*
 Applied Research Associates Inc. (c)2011

 Redistribution and use in source and binary forms,
   with or without modification, are permitted provided that the
   following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Applied Research Associates Inc nor the names
      of its contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#include <jni.h>
#include "physicseffects-android.h"
#include "../render_func.h"
#include "../perf_func.h"

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------------
//  Java_pfx_renderingsupport_PhysicsEffectsRenderer_nativePhysicsEffectsInit
//
/// Implementation of JNI function to initilize the simulation and start the
/// first scene.
///
/// @param  env     Pointer to JNI environment
//----------------------------------------------------------------------------
void
Java_pfx_renderingsupport_PhysicsEffectsRenderer_nativePhysicsEffectsInit( JNIEnv*  env )
{
    init();
    sceneChange();
}

//----------------------------------------------------------------------------
//  Java_pfx_renderingsupport_PhysicsEffectsRenderer_nativePhysicsEffectsSimulate
//
/// Implementation of JNI function to update the simulation and view parameters,
/// perform one simulation step, and redraw the scene.
///
/// @param  env     Pointer to JNI environment
//----------------------------------------------------------------------------
void
Java_pfx_renderingsupport_PhysicsEffectsRenderer_nativePhysicsEffectsSimulate( JNIEnv*  env )
{
	update();
	physics_simulate();
	render();
	perf_sync();
}

//----------------------------------------------------------------------------
//  Java_pfx_renderingsupport_PhysicsEffectsRenderer_nativePhysicsEffectsUpdate
//
/// Implementation of JNI function to update the camera view and simulation
/// parameters.
///
/// @param  env     Pointer to JNI environment
//----------------------------------------------------------------------------
void
Java_pfx_renderingsupport_PhysicsEffectsRenderer_nativePhysicsEffectsUpdate( JNIEnv*  env )
{
	update();
}

//----------------------------------------------------------------------------
//  Java_pfx_renderingsupport_PhysicsEffectsRenderer_nativePhysicsEffectsPhysStep
//
/// Implementation of JNI function to perform one simulation step.
///
/// @param  env     Pointer to JNI environment
//----------------------------------------------------------------------------
void
Java_pfx_renderingsupport_PhysicsEffectsRenderer_nativePhysicsEffectsPhysStep( JNIEnv*  env )
{
	physics_simulate();
}

//----------------------------------------------------------------------------
//  Java_pfx_renderingsupport_PhysicsEffectsRenderer_nativePhysicsEffectsRender
//
/// Implementation of JNI function to render the scene.
///
/// @param  env     Pointer to JNI environment
//----------------------------------------------------------------------------
void
Java_pfx_renderingsupport_PhysicsEffectsRenderer_nativePhysicsEffectsRender( JNIEnv*  env )
{
	render();
}

//----------------------------------------------------------------------------
//  Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeSceneChange
//
/// Implementation of JNI function to change to the next scene.
///
/// @param  env     Pointer to JNI environment
//----------------------------------------------------------------------------
void
Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeSceneChange( JNIEnv*  env )
{
    sceneChange();
}

//----------------------------------------------------------------------------
//  Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeCameraUp
//
/// Implementation of JNI function to tilt the camera up
///
/// @param  env     Pointer to JNI environment
//----------------------------------------------------------------------------
void
Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeCameraUp( JNIEnv*  env )
{
	float angX, angY, r;
	render_get_view_angle(angX, angY, r);

	angX -= 0.05f;
	if(angX < -1.4f) angX = -1.4f;
	if(angX > -0.01f) angX = -0.01f;

	render_set_view_angle(angX, angY, r);
}

//----------------------------------------------------------------------------
//  Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeCameraDown
//
/// Implementation of JNI function to tilt the camera down
///
/// @param  env     Pointer to JNI environment
//----------------------------------------------------------------------------
void
Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeCameraDown( JNIEnv*  env )
{
	float angX, angY, r;
	render_get_view_angle(angX, angY, r);

	angX += 0.05f;
	if(angX < -1.4f) angX = -1.4f;
	if(angX > -0.01f) angX = -0.01f;

	render_set_view_angle(angX, angY, r);
}

//----------------------------------------------------------------------------
//  Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeCameraLeft
//
/// Implementation of JNI function to tilt the camera left
///
/// @param  env     Pointer to JNI environment
//----------------------------------------------------------------------------
void
Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeCameraLeft( JNIEnv*  env )
{
	float angX, angY, r;
	render_get_view_angle(angX, angY, r);

	angY -= 0.05f;

	render_set_view_angle(angX, angY, r);
}

//----------------------------------------------------------------------------
//  Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeCameraRight
//
/// Implementation of JNI function to tilt the camera right
///
/// @param  env     Pointer to JNI environment
//----------------------------------------------------------------------------
void
Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeCameraRight( JNIEnv*  env )
{
	float angX, angY, r;
	render_get_view_angle(angX, angY, r);

	angY += 0.05f;

	render_set_view_angle(angX, angY, r);
}

//----------------------------------------------------------------------------
//  Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeCameraZoomOut
//
/// Implementation of JNI function to dolly the camera away from scene,
/// effectively zooming out.
///
/// @param  env     Pointer to JNI environment
//----------------------------------------------------------------------------
void
Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeCameraZoomOut( JNIEnv*  env )
{
	float angX, angY, r;
	render_get_view_angle(angX, angY, r);

	r *= 1.1f;
	if(r > 500.0f) r = 500.0f;

	render_set_view_angle(angX, angY, r);
}

//----------------------------------------------------------------------------
//  Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeCameraZoomIn
//
/// Implementation of JNI function to dolly the camera towards scene,
/// effectively zooming in.
///
/// @param  env     Pointer to JNI environment
//----------------------------------------------------------------------------
void
Java_pfx_renderingsupport_PhysicsEffectsGLSurfaceView_nativeCameraZoomIn( JNIEnv*  env )
{
	float angX, angY, r;
	render_get_view_angle(angX, angY, r);

	r *= 0.9f;
	if(r < 1.0f) r = 1.0f;

	render_set_view_angle(angX, angY, r);
}

#ifdef __cplusplus
}
#endif
