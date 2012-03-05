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
LOCAL_PATH := $(BULLET_PFX_ROOT_CYGWIN)

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
LOCAL_CFLAGS := $(LOCAL_C_INCLUDES:%=-I%) -DUSE_PTHREADS -mfpu=neon -mfloat-abi=softfp -pthread -DSCE_PFX_USE_SIMD_VECTORMATH

# apply these flags if needed 
# -ffast-math -funsafe-math-optimizations

# apply this to disable optimization
# TARGET_CFLAGS := $(TARGET_CFLAGS) -O0

# apply these 2 to turn on assembly output (*.c/*.cpp to *.s file)
#compile-cpp-source = $(eval $(call ev-compile-cpp-source,$1,$(1:%$(LOCAL_CPP_EXTENSION)=%.s)))
#TARGET_CFLAGS := $(TARGET_CFLAGS) -S

# Enable or disable NEON. Don't forget to apply, or not apply, -mfpu=neon and -mfloat-abi=softfp
# flags in addition, e.g., if this is true both of those need to be included in LOCAL_CFLAGS
# to avoid the possibility that ndk-build will "forget" to add them on some files
LOCAL_ARM_NEON := true
TARGET_CFLAGS := $(filter-out -ffpu=vfp,$(TARGET_CFLAGS))

# setup to build static library, libpfxlibrary.a
LOCAL_MODULE := libpfxlibrary
LOCAL_C_INCLUDES := \
        include \
        include/physics_effects/base_level \
        include/physics_effects/base_level/base \
        include/physics_effects/base_level/broadphase \
        include/physics_effects/base_level/collision \
        include/physics_effects/base_level/rigidbody \
        include/physics_effects/base_level/solver \
        include/physics_effects/base_level/sort \
        include/physics_effects/low_level \
        include/physics_effects/low_level/broadphase \
        include/physics_effects/low_level/collision \
        include/physics_effects/low_level/solver \
        include/physics_effects/low_level/sort \
        include/physics_effects/low_level/task \
        include/physics_effects/util \
        include/vecmath/neon \
        src/base_level/broadphase \
        src/base_level/collision \
        src/base_level/solver \
        src/base_level/sort \
        src/low_level/broadphase \
        src/low_level/collision \
        src/low_level/solver \
        src/low_level/sort \
        src/low_level/task \
        src/util \
        src

# Note that vectormath_neon_assembly_implementations.S is needed here in order to compile, link, and use the
# NEON version of the vectormath library
LOCAL_SRC_FILES := \
        src/base_level/broadphase/pfx_update_broadphase_proxy.cpp \
        src/base_level/collision/pfx_collidable.cpp \
        src/base_level/collision/pfx_contact_box_box.cpp \
        src/base_level/collision/pfx_contact_box_capsule.cpp \
        src/base_level/collision/pfx_contact_box_sphere.cpp \
        src/base_level/collision/pfx_contact_cache.cpp \
        src/base_level/collision/pfx_contact_capsule_capsule.cpp \
        src/base_level/collision/pfx_contact_capsule_sphere.cpp \
        src/base_level/collision/pfx_contact_large_tri_mesh.cpp \
        src/base_level/collision/pfx_contact_manifold.cpp \
        src/base_level/collision/pfx_contact_sphere_sphere.cpp \
        src/base_level/collision/pfx_contact_tri_mesh_box.cpp \
        src/base_level/collision/pfx_contact_tri_mesh_capsule.cpp \
        src/base_level/collision/pfx_contact_tri_mesh_convex.cpp \
        src/base_level/collision/pfx_contact_tri_mesh_cylinder.cpp \
        src/base_level/collision/pfx_contact_tri_mesh_sphere.cpp \
        src/base_level/collision/pfx_gjk_solver.cpp \
        src/base_level/collision/pfx_gjk_support_func.cpp \
        src/base_level/collision/pfx_intersect_ray_box.cpp \
        src/base_level/collision/pfx_intersect_ray_capsule.cpp \
        src/base_level/collision/pfx_intersect_ray_convex.cpp \
        src/base_level/collision/pfx_intersect_ray_cylinder.cpp \
        src/base_level/collision/pfx_intersect_ray_large_tri_mesh.cpp \
        src/base_level/collision/pfx_intersect_ray_sphere.cpp \
        src/base_level/collision/pfx_shape.cpp \
        src/base_level/collision/pfx_simplex_solver.cpp \
        src/base_level/solver/pfx_contact_constraint.cpp  \
        src/base_level/solver/pfx_joint_ball.cpp \
        src/base_level/solver/pfx_joint_fix.cpp \
        src/base_level/solver/pfx_joint_hinge.cpp \
        src/base_level/solver/pfx_joint_slider.cpp \
        src/base_level/solver/pfx_joint_swing_twist.cpp \
        src/base_level/solver/pfx_joint_universal.cpp \
		src/base_level/solver/pfx_constraint_row_solver_neon.cpp \
        src/base_level/sort/pfx_sort.cpp \
        src/low_level/broadphase/pfx_broadphase_single.cpp \
        src/low_level/collision/pfx_batched_ray_cast_single.cpp \
        src/low_level/collision/pfx_batched_ray_cast_parallel.cpp \
        src/low_level/collision/pfx_collision_detection_single.cpp \
        src/low_level/collision/pfx_collision_detection_parallel.cpp \
        src/low_level/collision/pfx_detect_collision_func.cpp \
        src/low_level/collision/pfx_intersect_ray_func.cpp \
        src/low_level/collision/pfx_island_generation.cpp \
        src/low_level/collision/pfx_ray_cast.cpp \
        src/low_level/collision/pfx_refresh_contacts_single.cpp \
        src/low_level/collision/pfx_refresh_contacts_parallel.cpp \
        src/low_level/solver/pfx_constraint_solver_single.cpp \
        src/low_level/solver/pfx_constraint_solver_parallel.cpp \
        src/low_level/solver/pfx_joint_constraint_func.cpp \
        src/low_level/solver/pfx_update_rigid_states_single.cpp \
        src/low_level/solver/pfx_update_rigid_states_parallel.cpp \
        src/low_level/sort/pfx_parallel_sort_single.cpp \
        src/low_level/task/pfx_task_manager_pthreads.cpp \
        src/low_level/task/pfx_sync_components_pthreads.cpp \
        src/util/pfx_mass.cpp \
        src/util/pfx_mesh_creator.cpp \
        include/vecmath/neon/vectormath_neon_assembly_implementations.S

LOCAL_LDLIBS := -L$(SYSROOT)/usr/lib -lGLESv1_CM -ldl -lm -llog

include $(BUILD_STATIC_LIBRARY)
