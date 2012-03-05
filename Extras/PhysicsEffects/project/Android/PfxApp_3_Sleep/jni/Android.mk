#
# Applied Research Associates Inc. (c)2011
#
# Redistribution and use in source and binary forms,
#   with or without modification, are permitted provided that the
#   following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Applied Research Associates Inc nor the names
#      of its contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
#

# NOTE: The local path here is defined based on an environment
#       variable that points to the root of the Bullet/PhysicsEffects
#       code base. The variable must be set in the Cygwin style, e.g.,
#       BULLET_PFX_ROOT_CYGWIN = /cygdrive/d/tools/PhysicsEffectsRootPath
#
#       The reason for *not* just setting local path := $(call my-dir)
#       is that we wish to access local source files that are *not*
#       located in a "jni" subfolder and *not* located in a subfolder
#       of the project folder. This enables us to have Eclipse and
#       Android NDK projects that fit neatly into the Bullet/PhysicsEffects
#       folder structure, enabling a multi platform code base, without
#       forcing the "jni" and subfolder structure that Android SDK/NDK
#       prefer for pure Android projects.
LOCAL_PATH:= $(BULLET_PFX_ROOT_CYGWIN)

# Copy the shared static library, libpfxlibrary.a, into the build
include $(CLEAR_VARS)
LOCAL_MODULE := pfxlibrary
LOCAL_SRC_FILES := project/Android/PfxLibrary/obj/local/armeabi-v7a/libpfxlibrary.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
# Warning: do not enable SIMD vector math functions or NEON for running
# in the Android NDK emulator. NEON requires a physical device
# with NEON support.
#
# Note that in theory we can let LOCAL_ARM_NEON := true determine
# that the flags -mfpu=neon and -mfloat-abi=softfp should be applied.
# However, in practice we have seen one case where the ndk-build
# script "forgets" these flags for the occasional *.cpp file, thus
# causing those files to access the std definition of vectormath
# classes, which are completely different, including different size.
# So, we explicitly apply these flags to avoid having a broken
# build.
#
# Include -DSCE_PFX_USE_SIMD_VECTORMATH to build for ARM NEON SIMD
# intrinsics, and leave it off to use NEON for scalar math but without
# SIMD. 

LOCAL_CFLAGS := $(LOCAL_C_INCLUDES:%=-I%) -mfpu=neon -mfloat-abi=softfp -DSCE_PFX_USE_SIMD_VECTORMATH

# apply these flags if needed 
# -ffast-math -funsafe-math-optimizations

# apply this to disable optimization
# TARGET_CFLAGS := $(TARGET_CFLAGS) -O0

# apply these 2 to turn on assembly output (*.c/*.cpp to *.s file)
# compile-cpp-source = $(eval $(call ev-compile-cpp-source,$1,$(1:%$(LOCAL_CPP_EXTENSION)=%.s)))
# TARGET_CFLAGS := $(TARGET_CFLAGS) -S

# specify static libraries to link in
LOCAL_STATIC_LIBRARIES := pfxlibrary

# Enable or disable NEON. Don't forget to apply, or not apply, -mfpu=neon and -mfloat-abi=softfp
# flags in addition, e.g., if this is true both of those need to be included in LOCAL_CFLAGS
# to avoid the possibility that ndk-build will "forget" to add them on some files
LOCAL_ARM_NEON := true
TARGET_CFLAGS := $(filter-out -ffpu=vfp,$(TARGET_CFLAGS))

LOCAL_MODULE    := PfxApp_3_Sleep
LOCAL_C_INCLUDES := \
        $(LOCAL_PATH)/include \
        $(LOCAL_PATH)sample/api_physics_effects/3_sleep \
        $(LOCAL_PATH)sample/api_physics_effects/common

LOCAL_SRC_FILES := \
        sample/api_physics_effects/common/jni/physicseffects-android.cpp \
        sample/api_physics_effects/3_sleep/main.cpp \
        sample/api_physics_effects/3_sleep/physics_func.cpp \
        sample/api_physics_effects/common/ctrl_func.android.cpp \
		sample/api_physics_effects/common/render_func.android.cpp \
        sample/api_physics_effects/common/perf_func.android.cpp

LOCAL_LDLIBS := -L$(SYSROOT)/usr/lib -lGLESv1_CM -ldl -lm -llog

include $(BUILD_SHARED_LIBRARY)


