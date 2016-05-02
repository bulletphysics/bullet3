LOCAL_PATH := ../../..

include $(CLEAR_VARS)

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

# setup to build static library
LOCAL_MODULE := libBullet

LOCAL_C_INCLUDES :=  $(LOCAL_PATH)/src

#find all the file recursively under jni/
FILE_LIST := $(wildcard \
		$(LOCAL_PATH)/src/LinearMath/*.cpp \
		$(LOCAL_PATH)/src/Bullet3Common/*.cpp \
		$(LOCAL_PATH)/src/BulletCollision/BroadphaseCollision/*.cpp \
		$(LOCAL_PATH)/src/BulletCollision/CollisionDispatch/*.cpp \
		$(LOCAL_PATH)/src/BulletCollision/CollisionShapes/*.cpp \
		$(LOCAL_PATH)/src/BulletCollision/NarrowPhaseCollision/*.cpp \
		$(LOCAL_PATH)/src/BulletDynamics/ConstraintSolver/*.cpp \
		$(LOCAL_PATH)/src/BulletDynamics/Dynamics/*.cpp \
		$(LOCAL_PATH)/src/BulletDynamics/Featherstone/*.cpp \
		$(LOCAL_PATH)/src/BulletDynamics/MLCPSolvers/*.cpp \
		$(LOCAL_PATH)/src/BulletDynamics/Vehicle/*.cpp \
		$(LOCAL_PATH)/src/BulletDynamics/Character/*.cpp \
		$(LOCAL_PATH)/src/BulletSoftBody/*.cpp \
		$(LOCAL_PATH)/src/BulletInverseDynamics/*.cpp \
		$(LOCAL_PATH)/src/BulletInverseDynamics/details/*.cpp \
		)
LOCAL_SRC_FILES := $(FILE_LIST:$(LOCAL_PATH)/%=%)

include $(BUILD_STATIC_LIBRARY)
