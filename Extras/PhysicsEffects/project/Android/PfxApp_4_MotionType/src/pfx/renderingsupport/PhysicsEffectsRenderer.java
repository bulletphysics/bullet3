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

package pfx.renderingsupport;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import android.opengl.GLSurfaceView;
import android.util.Log;

/**
 * Implementation of the renderer. It calls the native physicseffects code
 * to run on the init and drawframe allowing the sample to perform its simulation.
 * @author ahamilton
 *
 */
public class PhysicsEffectsRenderer implements GLSurfaceView.Renderer {
    boolean startup = true;
    boolean notEnded = true;
    static long startuptime;
    static long endtime;
    int frameCounter;

	/**
	 * Initilize the physics simulation after the GL surface is created.
	 *
	 */
	public void onSurfaceCreated(GL10 gl, EGLConfig config)
	{
    	Log.v("Init physics", "Initing physics");
        nativePhysicsEffectsInit();
        Log.v("Init physics", "Inited physics");
    }

	/**
	 * Set the viewport for a changed GL surface
	 *
	 */
	public void onSurfaceChanged(GL10 gl, int w, int h)
	{
    	gl.glViewport(0, 0, w, h);
    }

	/**
	 * Perform one simulation step and render the scene.
	 *
	 */
	public void onDrawFrame(GL10 gl)
	{
        if(startup){
        	startup = false;
        	frameCounter = 0;
        	startuptime = System.currentTimeMillis();
        	endtime = startuptime + 10000;                    // sets run time to be 30 seconds 
        }

        nativePhysicsEffectsSimulate();

    	frameCounter++;
		long lCurrentTime = System.currentTimeMillis();
    	if(lCurrentTime > endtime && notEnded) {
			float fFrameRate = (1000.0f * (float)frameCounter) / ((float)lCurrentTime - startuptime);
			Log.v("PHYSICS TIMING STUDY", "Number of Frames: " + Integer.toString(frameCounter));
			Log.v("PHYSICS TIMING STUDY", "Time ellapsed (milliseconds): " + Long.toString(lCurrentTime - startuptime));
    		Log.v("PHYSICS TIMING STUDY", "Bullet Frame Rate: " + Float.toString(fFrameRate) + "fps");
    		notEnded = false;
    	}
    }

    private static native void nativePhysicsEffectsInit();
    private static native void nativePhysicsEffectsSimulate();
    private static native void nativePhysicsEffectsUpdate();
    private static native void nativePhysicsEffectsPhysStep();
    private static native void nativePhysicsEffectsRender();
}
