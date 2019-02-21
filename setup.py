
from setuptools import find_packages
from sys import platform as _platform
import sys
import glob
import os


from distutils.core import setup
from distutils.extension import Extension
from distutils.util import get_platform
from glob import glob

# monkey-patch for parallel compilation
import multiprocessing
import multiprocessing.pool


def parallelCCompile(self, sources, output_dir=None, macros=None, include_dirs=None, debug=0, extra_preargs=None, extra_postargs=None, depends=None):
    # those lines are copied from distutils.ccompiler.CCompiler directly
    macros, objects, extra_postargs, pp_opts, build = self._setup_compile(output_dir, macros, include_dirs, sources, depends, extra_postargs)
    cc_args = self._get_cc_args(pp_opts, debug, extra_preargs)
    # parallel code
    N = 2*multiprocessing.cpu_count()# number of parallel compilations
    def _single_compile(obj):
        try: src, ext = build[obj]
        except KeyError: return
        newcc_args = cc_args
        if _platform == "darwin":
          if src.endswith('.cpp'):
            newcc_args = cc_args + ["-stdlib=libc++"]
        self._compile(obj, src, ext, newcc_args, extra_postargs, pp_opts)
    # convert to list, imap is evaluated on-demand
    pool = multiprocessing.pool.ThreadPool(N)
    list(pool.imap(_single_compile,objects))
    return objects
import distutils.ccompiler
distutils.ccompiler.CCompiler.compile=parallelCCompile

#see http://stackoverflow.com/a/8719066/295157
import os


platform = get_platform()
print(platform)

CXX_FLAGS = ''
CXX_FLAGS += '-DGWEN_COMPILE_STATIC '
CXX_FLAGS += '-DBT_USE_DOUBLE_PRECISION '
CXX_FLAGS += '-DBT_ENABLE_ENET '
CXX_FLAGS += '-DBT_ENABLE_CLSOCKET '
CXX_FLAGS += '-DB3_DUMP_PYTHON_VERSION '
CXX_FLAGS += '-DEGL_ADD_PYTHON_INIT '
CXX_FLAGS += '-DB3_ENABLE_FILEIO_PLUGIN '
CXX_FLAGS += '-DB3_USE_ZIPFILE_FILEIO '
CXX_FLAGS += '-DBT_THREADSAFE=1 '
CXX_FLAGS += '-DSTATIC_LINK_SPD_PLUGIN '
CXX_FLAGS += '-DPX_PHYSX_STATIC_LIB '
CXX_FLAGS += '-DBT_ENABLE_PHYSX '
CXX_FLAGS += '-DPX_FOUNDATION_DLL=0 '
CXX_FLAGS += '-DPX_COOKING '
CXX_FLAGS += '-DNDEBUG '
CXX_FLAGS += '-DDISABLE_CUDA_PHYSX '



EGL_CXX_FLAGS = ''



# libraries += [current_python]

libraries = []
include_dirs = ["examples", 
   "src/PhysX/physx/source/common/include",
   "src/PhysX/physx/source/common/src",
    "src/PhysX/physx/source/fastxml/include",
    "src/PhysX/physx/source/filebuf/include",
    "src/PhysX/physx/source/foundation/include",
    "src/PhysX/physx/source/geomutils/include",
    "src/PhysX/physx/source/geomutils/src",
    "src/PhysX/physx/source/geomutils/src/ccd",
    "src/PhysX/physx/source/geomutils/src/common",
    "src/PhysX/physx/source/geomutils/src/contact",
    "src/PhysX/physx/source/geomutils/src/convex",
    "src/PhysX/physx/source/geomutils/src/distance",
    "src/PhysX/physx/source/geomutils/src/gjk",
    "src/PhysX/physx/source/geomutils/src/hf",
    "src/PhysX/physx/source/geomutils/src/intersection",
    "src/PhysX/physx/source/geomutils/src/mesh",
    "src/PhysX/physx/source/geomutils/src/pcm",
    "src/PhysX/physx/source/geomutils/src/sweep",
    "src/PhysX/physx/source/lowlevel/api/include",
    "src/PhysX/physx/source/lowlevel/common/include",
    "src/PhysX/physx/source/lowlevel/common/include/collision",
    "src/PhysX/physx/source/lowlevel/common/include/pipeline",
    "src/PhysX/physx/source/lowlevel/common/include/utils",
    "src/PhysX/physx/source/lowlevel/software/include",
    "src/PhysX/physx/source/lowlevelaabb/include",
    "src/PhysX/physx/source/lowleveldynamics/include",
    "src/PhysX/physx/source/physx/src",
    "src/PhysX/physx/source/physx/src/buffering",
    "src/PhysX/physx/source/physx/src/device",
    "src/PhysX/physx/source/physxcooking/src",
    "src/PhysX/physx/source/physxcooking/src/convex",
    "src/PhysX/physx/source/physxcooking/src/mesh",
    "src/PhysX/physx/source/physxextensions/src",
    "src/PhysX/physx/source/physxextensions/src/serialization/Binary",
    "src/PhysX/physx/source/physxextensions/src/serialization/File",
    "src/PhysX/physx/source/physxextensions/src/serialization/Xml",
    "src/PhysX/physx/source/physxmetadata/core/include",
    "src/PhysX/physx/source/physxmetadata/extensions/include",
    "src/PhysX/physx/source/physxvehicle/src",
    "src/PhysX/physx/source/physxvehicle/src/physxmetadata/include",
    "src/PhysX/physx/source/pvd/include",
    "src/PhysX/physx/source/scenequery/include",
    "src/PhysX/physx/source/simulationcontroller/include",
    "src/PhysX/physx/source/simulationcontroller/src",
    "src/PhysX/physx/include",
    "src/PhysX/physx/include/characterkinematic",
    "src/PhysX/physx/include/common",
    "src/PhysX/physx/include/cooking",
    "src/PhysX/physx/include/extensions",
    "src/PhysX/physx/include/geometry",
    "src/PhysX/physx/include/geomutils",
    "src/PhysX/physx/include/vehicle",
    "src/PhysX/pxshared/include",
    ]
    

try:
    import numpy
    NP_DIRS = [numpy.get_include()]
except:
	  print("numpy is disabled. getCameraImage maybe slower.")
else:
	  print("numpy is enabled.")
	  CXX_FLAGS += '-DPYBULLET_USE_NUMPY '
	  for d in NP_DIRS:
	    print("numpy_include_dirs = %s" % d)
	  include_dirs += NP_DIRS

sources = ["examples/pybullet/pybullet.c"]\
+["examples/SharedMemory/physx/PhysXC_API.cpp"]\
+["examples/SharedMemory/physx/PhysXServerCommandProcessor.cpp"]\
+["examples/SharedMemory/physx/PhysXUrdfImporter.cpp"]\
+["examples/SharedMemory/physx/URDF2PhysX.cpp"]\
+["src/btLinearMathAll.cpp"]\
+["src/PhysXGeomUtilsAll.cpp"]\
+["examples/SharedMemory/plugins/eglPlugin/eglRendererVisualShapeConverter.cpp"]\
+["examples/SharedMemory/plugins/eglPlugin/eglRendererPlugin.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuFeatureCode.cpp"]\
+["src/PhysX/physx/source/geomutils/src/GuSweepMTD.cpp"]\
+["src/PhysX/physx/source/geomutils/src/GuSweepSharedTests.cpp"]\
+["src/PhysX/physx/source/geomutils/src/GuSweepTests.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactBoxBox.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactCapsuleBox.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactCapsuleCapsule.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactCapsuleConvex.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactCapsuleMesh.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactConvexConvex.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactConvexMesh.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactPlaneBox.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactPlaneCapsule.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactPlaneConvex.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactPolygonPolygon.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactSphereBox.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactSphereCapsule.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactSphereMesh.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactSpherePlane.cpp"]\
+["src/PhysX/physx/source/geomutils/src/contact/GuContactSphereSphere.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuBV4_BoxOverlap.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuBV4_CapsuleSweep.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuBV4_CapsuleSweepAA.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuBV4_OBBSweep.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuBV4_Raycast.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuBV4_SphereOverlap.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuBV4_SphereSweep.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuMeshQuery.cpp"]\
+["src/PhysX/physx/source/geomutils/src/hf/GuHeightField.cpp"]\
+["src/PhysX/physx/source/geomutils/src/hf/GuHeightFieldUtil.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuBV4.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuBV4Build.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuBV4_AABBSweep.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuMidphaseBV4.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuMidphaseRTree.cpp"]\
+["src/PhysX/physx/source/geomutils/src/sweep/GuSweepBoxBox.cpp"]\
+["src/PhysX/physx/source/geomutils/src/sweep/GuSweepBoxSphere.cpp"]\
+["src/PhysX/physx/source/geomutils/src/sweep/GuSweepBoxTriangle_FeatureBased.cpp"]\
+["src/PhysX/physx/source/geomutils/src/sweep/GuSweepBoxTriangle_SAT.cpp"]\
+["src/PhysX/physx/source/geomutils/src/sweep/GuSweepCapsuleBox.cpp"]\
+["src/PhysX/physx/source/geomutils/src/sweep/GuSweepCapsuleCapsule.cpp"]\
+["src/PhysX/physx/source/geomutils/src/sweep/GuSweepCapsuleTriangle.cpp"]\
+["src/PhysX/physx/source/geomutils/src/sweep/GuSweepSphereCapsule.cpp"]\
+["src/PhysX/physx/source/geomutils/src/sweep/GuSweepSphereSphere.cpp"]\
+["src/PhysX/physx/source/geomutils/src/sweep/GuSweepSphereTriangle.cpp"]\
+["src/PhysX/physx/source/geomutils/src/sweep/GuSweepTriangleUtils.cpp"]\
+["src/PhysX/physx/source/geomutils/src/pcm/GuPCMTriangleContactGen.cpp"]\
+["src/PhysX/physx/source/geomutils/src/pcm/GuPersistentContactManifold.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuBV32.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuBV32Build.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuOverlapTestsMesh.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuRTree.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuRTreeQueries.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuSweepsMesh.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuTriangleMesh.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuTriangleMeshBV4.cpp"]\
+["src/PhysX/physx/source/geomutils/src/mesh/GuTriangleMeshRTree.cpp"]\
+["src/PhysX/physx/source/geomutils/src/pcm/GuPCMContactBoxBox.cpp"]\
+["src/PhysX/physx/source/geomutils/src/pcm/GuPCMContactBoxConvex.cpp"]\
+["src/PhysX/physx/source/fastxml/src/PsFastXml.cpp"]\
+["src/PhysX/physx/source/immediatemode/src/NpImmediateMode.cpp"]\
+["src/PhysX/physx/source/physxmetadata/core/src/PxAutoGeneratedMetaDataObjects.cpp"]\
+["src/PhysX/physx/source/physxmetadata/core/src/PxMetaDataObjects.cpp"]\
+["src/PhysX/physx/source/physxmetadata/extensions/src/PxExtensionAutoGeneratedMetaDataObjects.cpp"]\
+["src/PhysX/physx/source/task/src/TaskManager.cpp"]\
+["src/PhysXVehicleAll.cpp"]\
+["src/PhysX/physx/source/pvd/src/PxPvdProfileZoneClient.cpp"]\
+["src/PhysX/physx/source/pvd/src/PxPvdUserRenderer.cpp"]\
+["src/PhysX/physx/source/pvd/src/PxPvdImpl.cpp"]\
+["src/PhysX/physx/source/pvd/src/PxPvdMemClient.cpp"]\
+["src/PhysX/physx/source/pvd/src/PxPvdObjectModelMetaData.cpp"]\
+["src/PhysX/physx/source/pvd/src/PxPvdObjectRegistrar.cpp"]\
+["src/PhysX/physx/source/pvd/src/PxProfileEventImpl.cpp"]\
+["src/PhysX/physx/source/pvd/src/PxPvd.cpp"]\
+["src/PhysX/physx/source/pvd/src/PxPvdDataStream.cpp"]\
+["src/PhysX/physx/source/pvd/src/PxPvdDefaultFileTransport.cpp"]\
+["src/PhysX/physx/source/pvd/src/PxPvdDefaultSocketTransport.cpp"]\
+["src/PhysX/physx/source/physxcooking/src/convex/ConvexHullBuilder.cpp"]\
+["src/PhysX/physx/source/physxcooking/src/convex/QuickHullConvexHullLib.cpp"]\
+["src/PhysX/physx/source/physxcooking/src/mesh/TriangleMeshBuilder.cpp"]\
+["src/PhysXCharacterAll.cpp"]\
+["src/PhysXCommonAll.cpp"]\
+["src/PhysXCookingAll.cpp"]\
+["src/PhysXExtensionAll.cpp"]\
+["src/PhysX/physx/source/physxextensions/src/ExtDefaultErrorCallback.cpp"]\
+["src/PhysX/physx/source/physxextensions/src/serialization/SnSerialization.cpp"]\
+["src/PhysX/physx/source/physxextensions/src/serialization/Binary/SnConvX.cpp"]\
+["src/PhysX/physx/source/physxextensions/src/serialization/Binary/SnConvX_Align.cpp"]\
+["src/PhysX/physx/source/physxextensions/src/serialization/Binary/SnConvX_Convert.cpp"]\
+["src/PhysX/physx/source/physxextensions/src/serialization/Binary/SnConvX_Error.cpp"]\
+["src/PhysX/physx/source/physxextensions/src/serialization/Binary/SnConvX_MetaData.cpp"]\
+["src/PhysX/physx/source/physxextensions/src/serialization/Binary/SnConvX_Output.cpp"]\
+["src/PhysX/physx/source/physxextensions/src/serialization/Binary/SnConvX_Union.cpp"]\
+["src/PhysX/physx/source/physxextensions/src/serialization/Binary/SnSerializationContext.cpp"]\
+["src/PhysX/physx/source/physxextensions/src/serialization/Xml/SnRepXUpgrader.cpp"]\
+["src/PhysX/physx/source/physxextensions/src/serialization/Xml/SnXmlSerialization.cpp"]\
+["src/PhysXFoundationAll.cpp"]\
+["src/PhysXLowLevelAll.cpp"]\
+["src/PhysXNpSrcAll.cpp"]\
+["src/PhysXSceneQueryAll.cpp"]\
+["src/PhysXSimulationControllerAll.cpp"]\
+["src/PhysX/physx/source/simulationcontroller/src/ScShapeSim.cpp"]\
+["src/PhysX/physx/source/lowleveldynamics/src/DyTGSContactPrep.cpp"]\
+["src/PhysX/physx/source/lowleveldynamics/src/DyTGSContactPrepBlock.cpp"]\
+["src/PhysX/physx/source/lowleveldynamics/src/DyTGSDynamics.cpp"]\
+["src/PhysX/physx/source/lowleveldynamics/src/DyTGSPartition.cpp"]\
+["src/PhysX/physx/source/lowlevelaabb/src/BpAABBManager.cpp"]\
+["src/PhysX/physx/source/lowlevelaabb/src/BpBroadPhase.cpp"]\
+["src/PhysX/physx/source/lowlevelaabb/src/BpBroadPhaseABP.cpp"]\
+["src/PhysX/physx/source/lowlevelaabb/src/BpBroadPhaseMBP.cpp"]\
+["src/PhysX/physx/source/lowlevelaabb/src/BpBroadPhaseSap.cpp"]\
+["src/PhysX/physx/source/lowlevelaabb/src/BpBroadPhaseSapAux.cpp"]\
+["src/PhysX/physx/source/lowlevelaabb/src/BpBroadPhaseShared.cpp"]\
+["src/PhysX/physx/source/lowlevelaabb/src/BpMBPTasks.cpp"]\
+["src/PhysX/physx/source/lowlevelaabb/src/BpSAPTasks.cpp"]\
+["src/btBulletCollisionAll.cpp"]\
+["src/btBulletDynamicsAll.cpp"]\
+["examples/ExampleBrowser/InProcessExampleBrowser.cpp"]\
+["examples/TinyRenderer/geometry.cpp"]\
+["examples/TinyRenderer/model.cpp"]\
+["examples/TinyRenderer/tgaimage.cpp"]\
+["examples/TinyRenderer/our_gl.cpp"]\
+["examples/TinyRenderer/TinyRenderer.cpp"]\
+["examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp"]\
+["examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.cpp"]\
+["examples/SharedMemory/plugins/fileIOPlugin/fileIOPlugin.cpp"]\
+["examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp"]\
+["examples/SharedMemory/IKTrajectoryHelper.cpp"]\
+["examples/SharedMemory/InProcessMemory.cpp"]\
+["examples/SharedMemory/PhysicsClient.cpp"]\
+["examples/SharedMemory/PhysicsServer.cpp"]\
+["examples/SharedMemory/PhysicsServerExample.cpp"]\
+["examples/SharedMemory/PhysicsServerExampleBullet2.cpp"]\
+["examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp"]\
+["examples/SharedMemory/PhysicsServerSharedMemory.cpp"]\
+["examples/SharedMemory/PhysicsDirect.cpp"]\
+["examples/SharedMemory/PhysicsDirectC_API.cpp"]\
+["examples/SharedMemory/PhysicsServerCommandProcessor.cpp"]\
+["examples/SharedMemory/PhysicsClientSharedMemory.cpp"]\
+["examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp"]\
+["examples/SharedMemory/PhysicsClientC_API.cpp"]\
+["examples/SharedMemory/Win32SharedMemory.cpp"]\
+["examples/SharedMemory/PosixSharedMemory.cpp"]\
+["examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp"]\
+["examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp"]\
+["examples/SharedMemory/plugins/stablePDPlugin/BulletConversion.cpp"]\
+["examples/SharedMemory/plugins/stablePDPlugin/KinTree.cpp"]\
+["examples/SharedMemory/plugins/stablePDPlugin/MathUtil.cpp"]\
+["examples/SharedMemory/plugins/stablePDPlugin/RBDModel.cpp"]\
+["examples/SharedMemory/plugins/stablePDPlugin/RBDUtil.cpp"]\
+["examples/SharedMemory/plugins/stablePDPlugin/Shape.cpp"]\
+["examples/SharedMemory/plugins/stablePDPlugin/SpAlg.cpp"]\
+["examples/SharedMemory/PhysicsClientUDP.cpp"]\
+["examples/SharedMemory/PhysicsClientUDP_C_API.cpp"]\
+["examples/SharedMemory/PhysicsClientTCP.cpp"]\
+["examples/SharedMemory/PhysicsClientTCP_C_API.cpp"]\
+["examples/SharedMemory/b3PluginManager.cpp"]\
+["examples/Utils/b3ResourcePath.cpp"]\
+["examples/Utils/RobotLoggingUtil.cpp"]\
+["examples/Utils/ChromeTraceUtil.cpp"]\
+["examples/Utils/b3Clock.cpp"]\
+["examples/Utils/b3Quickprof.cpp"]\
+["examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp"]\
+["examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp"]\
+["examples/ThirdPartyLibs/stb_image/stb_image.cpp"]\
+["examples/ThirdPartyLibs/stb_image/stb_image_write.cpp"]\
+["examples/ThirdPartyLibs/minizip/ioapi.c"]\
+["examples/ThirdPartyLibs/minizip/unzip.c"]\
+["examples/ThirdPartyLibs/minizip/zip.c"]\
+["examples/ThirdPartyLibs/zlib/adler32.c"]\
+["examples/ThirdPartyLibs/zlib/compress.c"]\
+["examples/ThirdPartyLibs/zlib/crc32.c"]\
+["examples/ThirdPartyLibs/zlib/deflate.c"]\
+["examples/ThirdPartyLibs/zlib/gzclose.c"]\
+["examples/ThirdPartyLibs/zlib/gzlib.c"]\
+["examples/ThirdPartyLibs/zlib/gzread.c"]\
+["examples/ThirdPartyLibs/zlib/gzwrite.c"]\
+["examples/ThirdPartyLibs/zlib/infback.c"]\
+["examples/ThirdPartyLibs/zlib/inffast.c"]\
+["examples/ThirdPartyLibs/zlib/inflate.c"]\
+["examples/ThirdPartyLibs/zlib/inftrees.c"]\
+["examples/ThirdPartyLibs/zlib/trees.c"]\
+["examples/ThirdPartyLibs/zlib/uncompr.c"]\
+["examples/ThirdPartyLibs/zlib/zutil.c"]\
+["examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp"]\
+["examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp"]\
+["examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp"]\
+["examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp"]\
+["examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp"]\
+["examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp"]\
+["examples/Importers/ImportURDFDemo/URDF2Bullet.cpp"]\
+["examples/Importers/ImportURDFDemo/UrdfParser.cpp"]\
+["examples/Importers/ImportURDFDemo/urdfStringSplit.cpp"]\
+["examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp"]\
+["examples/MultiThreading/b3PosixThreadSupport.cpp"]\
+["examples/MultiThreading/b3Win32ThreadSupport.cpp"]\
+["examples/MultiThreading/b3ThreadSupportInterface.cpp"]\
+["examples/ThirdPartyLibs/enet/callbacks.c"]\
+["examples/ThirdPartyLibs/enet/compress.c"]\
+["examples/ThirdPartyLibs/enet/host.c"]\
+["examples/ThirdPartyLibs/enet/list.c"]\
+["examples/ThirdPartyLibs/enet/packet.c"]\
+["examples/ThirdPartyLibs/enet/peer.c"]\
+["examples/ThirdPartyLibs/enet/protocol.c"]\
+["examples/ExampleBrowser/OpenGLGuiHelper.cpp"]\
+["examples/ExampleBrowser/OpenGLExampleBrowser.cpp"]\
+["examples/ExampleBrowser/CollisionShape2TriangleMesh.cpp"]\
+["examples/ExampleBrowser/GL_ShapeDrawer.cpp"]\
+["examples/OpenGLWindow/SimpleOpenGL2Renderer.cpp"]\
+["examples/OpenGLWindow/GLInstancingRenderer.cpp"]\
+["examples/OpenGLWindow/SimpleOpenGL3App.cpp"]\
+["examples/OpenGLWindow/GLPrimitiveRenderer.cpp"]\
+["examples/OpenGLWindow/TwFonts.cpp"]\
+["examples/OpenGLWindow/GLRenderToTexture.cpp"]\
+["examples/OpenGLWindow/LoadShader.cpp"]\
+["examples/OpenGLWindow/OpenSans.cpp"]\
+["examples/OpenGLWindow/SimpleCamera.cpp"]\
+["examples/OpenGLWindow/fontstash.cpp"]\
+["examples/OpenGLWindow/SimpleOpenGL2App.cpp"]\
+["examples/OpenGLWindow/opengl_fontstashcallbacks.cpp"]\
+["examples/ExampleBrowser/GwenGUISupport/GraphingTexture.cpp"]\
+["examples/ExampleBrowser/GwenGUISupport/GwenProfileWindow.cpp"]\
+["examples/ExampleBrowser/GwenGUISupport/gwenUserInterface.cpp"]\
+["examples/ExampleBrowser/GwenGUISupport/GwenParameterInterface.cpp"]\
+["examples/ExampleBrowser/GwenGUISupport/GwenTextureWindow.cpp"]\
+["src/Bullet3Common/b3AlignedAllocator.cpp"]\
+["src/Bullet3Common/b3Logging.cpp"]\
+["src/Bullet3Common/b3Vector3.cpp"]\
+["examples/ThirdPartyLibs/clsocket/src/ActiveSocket.cpp"]\
+["examples/ThirdPartyLibs/clsocket/src/PassiveSocket.cpp"]\
+["examples/ThirdPartyLibs/clsocket/src/SimpleSocket.cpp"]\
+["Extras/Serialize/BulletFileLoader/bChunk.cpp"]\
+["Extras/Serialize/BulletFileLoader/bDNA.cpp"]\
+["Extras/Serialize/BulletFileLoader/bFile.cpp"]\
+["Extras/Serialize/BulletFileLoader/btBulletFile.cpp"]\
+["Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.cpp"]\
+["Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp"]\
+["Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp"]\
+["Extras/InverseDynamics/CloneTreeCreator.cpp"]\
+["Extras/InverseDynamics/IDRandomUtil.cpp"]\
+["Extras/InverseDynamics/MultiBodyTreeDebugGraph.cpp"]\
+["Extras/InverseDynamics/User2InternalIndex.cpp"]\
+["Extras/InverseDynamics/CoilCreator.cpp"]\
+["Extras/InverseDynamics/MultiBodyNameMap.cpp"]\
+["Extras/InverseDynamics/RandomTreeCreator.cpp"]\
+["Extras/InverseDynamics/btMultiBodyTreeCreator.cpp"]\
+["Extras/InverseDynamics/DillCreator.cpp"]\
+["Extras/InverseDynamics/MultiBodyTreeCreator.cpp"]\
+["Extras/InverseDynamics/SimpleTreeCreator.cpp"]\
+["Extras/InverseDynamics/invdyn_bullet_comparison.cpp"]\
+["src/BulletSoftBody/btDefaultSoftBodySolver.cpp"]\
+["src/BulletSoftBody/btSoftBodyHelpers.cpp"]\
+["src/BulletSoftBody/btSoftRigidCollisionAlgorithm.cpp"]\
+["src/BulletSoftBody/btSoftBody.cpp"]\
+["src/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.cpp"]\
+["src/BulletSoftBody/btSoftRigidDynamicsWorld.cpp"]\
+["src/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.cpp"]\
+["src/BulletSoftBody/btSoftMultiBodyDynamicsWorld.cpp"]\
+["src/BulletSoftBody/btSoftSoftCollisionAlgorithm.cpp"]\
+["src/BulletInverseDynamics/IDMath.cpp"]\
+["src/BulletInverseDynamics/MultiBodyTree.cpp"]\
+["src/BulletInverseDynamics/details/MultiBodyTreeImpl.cpp"]\
+["src/BulletInverseDynamics/details/MultiBodyTreeInitCache.cpp"]\
+["examples/ThirdPartyLibs/BussIK/Jacobian.cpp"]\
+["examples/ThirdPartyLibs/BussIK/LinearR2.cpp"]\
+["examples/ThirdPartyLibs/BussIK/LinearR3.cpp"]\
+["examples/ThirdPartyLibs/BussIK/LinearR4.cpp"]\
+["examples/ThirdPartyLibs/BussIK/MatrixRmn.cpp"]\
+["examples/ThirdPartyLibs/BussIK/Misc.cpp"]\
+["examples/ThirdPartyLibs/BussIK/Node.cpp"]\
+["examples/ThirdPartyLibs/BussIK/Tree.cpp"]\
+["examples/ThirdPartyLibs/BussIK/VectorRn.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Anim.cpp"]\
+["examples/ThirdPartyLibs/Gwen/DragAndDrop.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Hook.cpp"]\
+["examples/ThirdPartyLibs/Gwen/ToolTip.cpp"]\
+["examples/ThirdPartyLibs/Gwen/events.cpp"]\
+["examples/ThirdPartyLibs/Gwen/BaseRender.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Gwen.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Skin.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Utility.cpp"]\
+["examples/ThirdPartyLibs/Gwen/inputhandler.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/Base.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/Button.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/Canvas.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/CheckBox.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/ColorControls.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/ColorPicker.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/ComboBox.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/CrossSplitter.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/DockBase.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/DockedTabControl.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/Dragger.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/GroupBox.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/HSVColorPicker.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/HorizontalScrollBar.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/ImagePanel.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/HorizontalSlider.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/Label.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/LabelClickable.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/ListBox.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/MenuItem.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/Menu.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/MenuStrip.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/NumericUpDown.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/PanelListPanel.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/ProgressBar.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/Properties.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/RadioButton.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/RadioButtonController.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/ResizableControl.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/Resizer.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/RichLabel.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/ScrollBar.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/ScrollBarBar.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/ScrollBarButton.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/ScrollControl.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/Slider.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/SplitterBar.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/TabButton.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/TabControl.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/TabStrip.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/Text.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/TextBox.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/TextBoxNumeric.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/TreeControl.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/TreeNode.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/VerticalScrollBar.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/VerticalSlider.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/WindowControl.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/Dialog/FileOpen.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/Dialog/FileSave.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Controls/Dialog/Query.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Platforms/Null.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Platforms/Windows.cpp"]\
+["examples/ThirdPartyLibs/Gwen/Renderers/OpenGL_DebugFont.cpp"]



egl_renderer_sources = ["examples/SharedMemory/plugins/eglPlugin/eglRendererVisualShapeConverter.cpp"]\
+["examples/SharedMemory/plugins/eglPlugin/eglRendererPlugin.cpp"]\
+["examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp"]\
+["examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp"]\
+["examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp"]\
+["examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp"]\
+["examples/TinyRenderer/geometry.cpp"]\
+["examples/TinyRenderer/model.cpp"]\
+["examples/TinyRenderer/tgaimage.cpp"]\
+["examples/TinyRenderer/our_gl.cpp"]\
+["examples/TinyRenderer/TinyRenderer.cpp"]\
+["examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp"]\
+["examples/ThirdPartyLibs/stb_image/stb_image.cpp"]\
+["examples/ThirdPartyLibs/stb_image/stb_image_write.cpp"]\
+["examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp"]\
+["examples/OpenGLWindow/SimpleCamera.cpp"]\
+["examples/Utils/b3Clock.cpp"]\
+["examples/Utils/b3ResourcePath.cpp"]\
+["src/BulletCollision/CollisionShapes/btShapeHull.cpp"]\
+["src/BulletCollision/CollisionShapes/btConvexHullShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btBoxShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btSphereShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btConvexShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btCollisionShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp"]\
+["src/BulletCollision/CollisionShapes/btConvexInternalShape.cpp"]\
+["src/Bullet3Common/b3Logging.cpp"]\
+["src/LinearMath/btAlignedAllocator.cpp"]\
+["src/LinearMath/btConvexHull.cpp"]\
+["src/LinearMath/btConvexHullComputer.cpp"] \
+["src/LinearMath/btGeometryUtil.cpp"]\
+["src/LinearMath/btQuickprof.cpp"] \
+["src/LinearMath/btThreads.cpp"] \
+["src/Bullet3Common/b3AlignedAllocator.cpp"] \
+["examples/ThirdPartyLibs/glad/gl.c"]\
+["examples/OpenGLWindow/GLInstancingRenderer.cpp"]\
+["examples/OpenGLWindow/GLRenderToTexture.cpp"] \
+["examples/OpenGLWindow/LoadShader.cpp"]

if 'BT_USE_EGL' in CXX_FLAGS:
    sources += ['examples/ThirdPartyLibs/glad/egl.c']
    sources += ['examples/OpenGLWindow/EGLOpenGLWindow.cpp']

if _platform == "linux" or _platform == "linux2":
    libraries = ['dl','pthread']
    CXX_FLAGS += '-D_LINUX '
    CXX_FLAGS += '-DGLEW_STATIC '
    CXX_FLAGS += '-DGLEW_INIT_OPENGL11_FUNCTIONS=1 '
    CXX_FLAGS += '-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1 '
    CXX_FLAGS += '-DDYNAMIC_LOAD_X11_FUNCTIONS '
    CXX_FLAGS += '-DHAS_SOCKLEN_T '
    CXX_FLAGS += '-fno-inline-functions-called-once '
    EGL_CXX_FLAGS += '-DBT_USE_EGL '
    EGL_CXX_FLAGS += '-fPIC ' # for plugins

    sources = sources + ["examples/ThirdPartyLibs/enet/unix.c"]\
    +["examples/OpenGLWindow/X11OpenGLWindow.cpp"]\
    +["examples/ThirdPartyLibs/glad/gl.c"]\
    +["src/PhysXFoundationUnix.cpp"]\
    +["examples/ThirdPartyLibs/glad/glx.c"]
    include_dirs += ["examples/ThirdPartyLibs/optionalX11"]

    if 'BT_USE_EGL' in EGL_CXX_FLAGS:
        egl_renderer_sources = egl_renderer_sources\
        +["examples/OpenGLWindow/EGLOpenGLWindow.cpp"]\
        +['examples/ThirdPartyLibs/glad/egl.c']
    else:
        egl_renderer_sources = egl_renderer_sources\
        +["examples/OpenGLWindow/X11OpenGLWindow.cpp"]\
        +["examples/ThirdPartyLibs/glad/glx.c"]


elif _platform == "win32":
    print("win32!")
    libraries = ['Ws2_32','Winmm','User32','Opengl32','kernel32','glu32','Gdi32','Comdlg32']
    CXX_FLAGS += '-DWIN32 '
    CXX_FLAGS += '-DGLEW_STATIC '
    sources = sources + ["examples/ThirdPartyLibs/enet/win32.c"]\
    +["examples/OpenGLWindow/Win32Window.cpp"]\
    +["examples/OpenGLWindow/Win32OpenGLWindow.cpp"]\
    +["examples/ThirdPartyLibs/glad/gl.c"]
    sources = sources + +["src/PhysX/physx/source/physx/src/windows/NpWindowsDelayLoadHook.cpp"]\
    +["src/PhysX/physx/source/common/src/windows/CmWindowsDelayLoadHook.cpp"]\
    +["PhysX/physx/source/common/src/windows/CmWindowsModuleUpdateLoader.cpp"]\
    +["src/PhysXFoundationWindows.cpp"]\
    +["src/PhysX/physx/source/physx/src/device/windows/PhysXIndicatorWindows.cpp"]
elif _platform == "darwin":
    print("darwin!")
    os.environ['LDFLAGS'] = '-framework Cocoa -stdlib=libc++ -framework OpenGL'
    CXX_FLAGS += '-DB3_NO_PYTHON_FRAMEWORK '
    CXX_FLAGS += '-DHAS_SOCKLEN_T '
    CXX_FLAGS += '-D_DARWIN '
#    CXX_FLAGS += '-framework Cocoa '
    sources = sources + ["examples/ThirdPartyLibs/enet/unix.c"]\
    +["src/PhysXFoundationUnix.cpp"]\
    +["examples/OpenGLWindow/MacOpenGLWindow.cpp"]\
    +["examples/ThirdPartyLibs/glad/gl.c"]\
    +["examples/OpenGLWindow/MacOpenGLWindowObjC.m"]
else:
    print("bsd!")
    libraries = ['GL','GLEW','pthread']
    os.environ['LDFLAGS'] = '-L/usr/X11R6/lib'
    CXX_FLAGS += '-D_BSD '
    CXX_FLAGS += '-I/usr/X11R6/include '
    CXX_FLAGS += '-DHAS_SOCKLEN_T '
    CXX_FLAGS += '-fno-inline-functions-called-once'
    sources = ["examples/ThirdPartyLibs/enet/unix.c"]\
    +["src/PhysXFoundationUnix.cpp"]\
    +["examples/OpenGLWindow/X11OpenGLWindow.cpp"]\
    +["examples/ThirdPartyLibs/glad/gl.c"]\
    + sources

setup_py_dir = os.path.dirname(os.path.realpath(__file__))

need_files = []
datadir = "examples/pybullet/gym/pybullet_data"

hh = setup_py_dir + "/" + datadir

for root, dirs, files in os.walk(hh):
    for fn in files:
        ext = os.path.splitext(fn)[1][1:]
        if ext and ext in 'yaml index meta data-00000-of-00001 png gif jpg urdf sdf obj txt mtl dae off stl STL xml '.split():
            fn = root + "/" + fn
            need_files.append(fn[1+len(hh):])

print("found resource files: %i" % len(need_files))
for n in need_files: print("-- %s" % n)
print("packages")
print(find_packages('examples/pybullet/gym'))
print("-----")

extensions = []

pybullet_ext = Extension("pybullet",
        sources =  sources,
        libraries = libraries,
        extra_compile_args=CXX_FLAGS.split(),
        include_dirs = include_dirs + ["src","examples/ThirdPartyLibs","examples/ThirdPartyLibs/glad", "examples/ThirdPartyLibs/enet/include","examples/ThirdPartyLibs/clsocket/src"]
     )
extensions.append(pybullet_ext)


if 'BT_USE_EGL' in EGL_CXX_FLAGS:

	eglRender = Extension("eglRenderer",
        	sources =  egl_renderer_sources,
        	libraries = libraries,
        	extra_compile_args=(CXX_FLAGS+EGL_CXX_FLAGS ).split(),
        	include_dirs = include_dirs + ["src","examples", "examples/ThirdPartyLibs","examples/ThirdPartyLibs/glad", "examples/ThirdPartyLibs/enet/include","examples/ThirdPartyLibs/clsocket/src"])

	extensions.append(eglRender)


setup(
	name = 'pybullet',
	version='2.4.3',
	description='Official Python Interface for the Bullet Physics SDK specialized for Robotics Simulation and Reinforcement Learning',
	long_description='pybullet is an easy to use Python module for physics simulation, robotics and deep reinforcement learning based on the Bullet Physics SDK. With pybullet you can load articulated bodies from URDF, SDF and other file formats. pybullet provides forward dynamics simulation, inverse dynamics computation, forward and inverse kinematics and collision detection and ray intersection queries. Aside from physics simulation, pybullet supports to rendering, with a CPU renderer and OpenGL visualization and support for virtual reality headsets.',
	url='https://github.com/bulletphysics/bullet3',
	author='Erwin Coumans, Yunfei Bai, Jasmine Hsu',
	author_email='erwincoumans@google.com',
	license='zlib',
	platforms='any',
	keywords=['game development', 'virtual reality', 'physics simulation', 'robotics', 'collision detection', 'opengl'],
	ext_modules = extensions,
	classifiers=['Development Status :: 5 - Production/Stable',
                   'License :: OSI Approved :: zlib/libpng License',
                   'Operating System :: Microsoft :: Windows',
                   'Operating System :: POSIX :: Linux',
                   'Operating System :: MacOS',
                   'Intended Audience :: Science/Research',
                   "Programming Language :: Python",
                   'Programming Language :: Python :: 2.7',
                   'Programming Language :: Python :: 3.4',
                   'Programming Language :: Python :: 3.5',
                   'Programming Language :: Python :: 3.6',
                   'Topic :: Games/Entertainment :: Simulation',
                   'Topic :: Scientific/Engineering :: Artificial Intelligence',
                   'Framework :: Robot Framework'],
    package_dir = { '': 'examples/pybullet/gym'},
    packages=[x for x in find_packages('examples/pybullet/gym')],
    package_data = { 'pybullet_data': need_files }
)
