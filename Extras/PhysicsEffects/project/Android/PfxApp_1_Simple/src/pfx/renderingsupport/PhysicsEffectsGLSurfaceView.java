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

import android.opengl.GLSurfaceView;
import android.content.Context;
import android.view.MotionEvent;

/**
 * Implementation of the OpenGL ES view window and creates/assigns a renderer.
 * Handles touch events to allow the user to change scene on tap, move the camera
 * around by touch based movement, and allows the user to zoom in and out by pinching.
 * @author ahamilton
 *
 */
public class PhysicsEffectsGLSurfaceView extends GLSurfaceView {
    public PhysicsEffectsGLSurfaceView(Context context) {
        super(context);
        peRenderer = new PhysicsEffectsRenderer();
        setRenderer(peRenderer);
    }
    
    float x1 = 0, x2, y1 = 0, y2;
    long downTime;

	/**
	 * Function to process touch events for camera and scene control
	 *
	 */
    public boolean onTouchEvent(final MotionEvent event) {		  
    	switch(event.getAction()) {
    		case(MotionEvent.ACTION_DOWN):
    		     x1 = event.getX();
    		     y1 = event.getY();
    		     downTime = event.getDownTime();
    		     break;
    		case(MotionEvent.ACTION_UP):
   		     	if(event.getEventTime() - downTime < 100) nativeSceneChange(); 
   		     	break;
    		case(MotionEvent.ACTION_MOVE): {
    			if(event.getPointerCount() == 1){
    				x2 = event.getX();
    				y2 = event.getY();
    				float dx = x2-x1;
    				float dy = y2-y1;

    				// Use dx and dy to determine the direction
    				if(dx > 0) {
    					nativeCameraLeft();
    				} else {  
    					nativeCameraRight();
    				} 
    				if(dy > 0){
    					nativeCameraUp();
    				} else {
    					nativeCameraDown();
    				}
    				x1=x2;
    				y1=y2;
    			} else if(event.getPointerCount() == 2){
       		     	y2 = event.getY();

       		     	float dy = y2-y1;

       		     	// Use dy to determine zoom
       		     	if(dy > 0){
       		     		nativeCameraZoomIn();
       		     	} else {
       		     		nativeCameraZoomOut();
       		     	}
       		     	y1=y2;
    			}
    		}
    	}
        return true;
    }

    PhysicsEffectsRenderer peRenderer;

    private static native void nativeCameraUp();
    private static native void nativeCameraDown();
    private static native void nativeCameraLeft();
    private static native void nativeCameraRight();
    private static native void nativeCameraZoomOut();
    private static native void nativeCameraZoomIn();
    private static native void nativeSceneChange();
}
