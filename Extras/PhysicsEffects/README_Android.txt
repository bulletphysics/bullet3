PhysicsEffects can be built and run on Android devices. The Android projects are
a mix of Java and C++ code. The code has been tested using Android NDK r5c. Build files
are provided for the NDK and ANT. Here are some instructions on how
to update the baseline PhysicsEffects branch of Bullet to support Android using the provided
files:

1. For Windows platforms, download and install Cygwin (if it is not already installed)
2. Download and install the Android SDK.
3. Download and install the Android NDK. (The files provided have been tested with NDK r5c.)

4. Set up these new environment variables:

    BULLET_PFX_ROOT_CYGWIN = path to your PhysicsEffects folder, for example:
    BULLET_PFX_ROOT_CYGWIN=/cygdrive/d/tools/Bullet/Extras/PhysicsEffects

    CYGWIN_ROOT = path to where Cygwin is installed (Windows-style path)
    ANDROID_NDK_ROOT = path to where the Android NDK is installed

5. Open a Cygwin console. Browse to the PhysicsEffects/project/Android
      folder. Then, for each package, starting with PfxLibrary, go into the
      package folder and run the ndk-build command. For example:

      /PhysicsEffects/project/Android/PfxLibrary> /cygwin/d/tools/android-ndk-r5c/ndk-build

      Build every package using ndk-build.

      (It is possible to configure Eclipse to use ndk-build, but that is somewhat
      tedious to do.)

6. For each application package (PfxApp*) after running ndk-build use
	    ant compile
	    ant debug

	 If your Android device is connected and setup, you can also run the app using
	    ant install

7. If you run the app, you can interact with the touch interface:

      - Tap the screen to change scenes
      - Pinch the screen to zoom in/out
      - Drag the screen to rotate the camera
