
from setuptools import find_packages
from sys import platform as _platform
import sys
import glob

from distutils.core import setup
from distutils.extension import Extension
from distutils.util import get_platform
from glob import glob

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



# libraries += [current_python]

libraries = []
include_dirs = []

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
+["examples/ExampleBrowser/InProcessExampleBrowser.cpp"]\
+["examples/TinyRenderer/geometry.cpp"]\
+["examples/TinyRenderer/model.cpp"]\
+["examples/TinyRenderer/tgaimage.cpp"]\
+["examples/TinyRenderer/our_gl.cpp"]\
+["examples/TinyRenderer/TinyRenderer.cpp"]\
+["examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp"]\
+["examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.cpp"]\
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
+["src/LinearMath/btAlignedAllocator.cpp"]\
+["src/LinearMath/btGeometryUtil.cpp"]\
+["src/LinearMath/btSerializer.cpp"]\
+["src/LinearMath/btVector3.cpp"]\
+["src/LinearMath/btConvexHull.cpp"]\
+["src/LinearMath/btPolarDecomposition.cpp"]\
+["src/LinearMath/btSerializer64.cpp"]\
+["src/LinearMath/btConvexHullComputer.cpp"]\
+["src/LinearMath/btQuickprof.cpp"]\
+["src/LinearMath/btThreads.cpp"]\
+["src/LinearMath/TaskScheduler/btTaskScheduler.cpp"]\
+["src/LinearMath/TaskScheduler/btThreadSupportPosix.cpp"]\
+["src/LinearMath/TaskScheduler/btThreadSupportWin32.cpp"]\
+["src/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp"]\
+["src/BulletCollision/BroadphaseCollision/btDbvt.cpp"]\
+["src/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp"]\
+["src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp"]\
+["src/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp"]\
+["src/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp"]\
+["src/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp"]\
+["src/BulletCollision/BroadphaseCollision/btDispatcher.cpp"]\
+["src/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp"]\
+["src/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp"]\
+["src/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp"]\
+["src/BulletCollision/CollisionDispatch/btHashedSimplePairCache.cpp"]\
+["src/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp"]\
+["src/BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.cpp"]\
+["src/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp"]\
+["src/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp"]\
+["src/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp"]\
+["src/BulletCollision/CollisionDispatch/btManifoldResult.cpp"]\
+["src/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp"]\
+["src/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp"]\
+["src/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp"]\
+["src/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp"]\
+["src/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp"]\
+["src/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp"]\
+["src/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp"]\
+["src/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp"]\
+["src/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp"]\
+["src/BulletCollision/CollisionDispatch/btCollisionObject.cpp"]\
+["src/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp"]\
+["src/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp"]\
+["src/BulletCollision/CollisionDispatch/btCollisionWorld.cpp"]\
+["src/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp"]\
+["src/BulletCollision/CollisionDispatch/btUnionFind.cpp"]\
+["src/BulletCollision/CollisionDispatch/btCollisionWorldImporter.cpp"]\
+["src/BulletCollision/CollisionDispatch/btGhostObject.cpp"]\
+["src/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp"]\
+["src/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp"]\
+["src/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp"]\
+["src/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp"]\
+["src/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp"]\
+["src/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp"]\
+["src/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp"]\
+["src/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp"]\
+["src/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp"]\
+["src/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp"]\
+["src/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp"]\
+["src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp"]\
+["src/BulletCollision/CollisionShapes/btBox2dShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp"]\
+["src/BulletCollision/CollisionShapes/btShapeHull.cpp"]\
+["src/BulletCollision/CollisionShapes/btBoxShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btConvexShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btSphereShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btCapsuleShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btCylinderShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp"]\
+["src/BulletCollision/CollisionShapes/btCollisionShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btEmptyShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btTetrahedronShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btCompoundShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btTriangleBuffer.cpp"]\
+["src/BulletCollision/CollisionShapes/btConcaveShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btTriangleCallback.cpp"]\
+["src/BulletCollision/CollisionShapes/btConeShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btMultiSphereShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp"]\
+["src/BulletCollision/CollisionShapes/btConvex2dShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp"]\
+["src/BulletCollision/CollisionShapes/btConvexHullShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btOptimizedBvh.cpp"]\
+["src/BulletCollision/CollisionShapes/btTriangleMesh.cpp"]\
+["src/BulletCollision/CollisionShapes/btConvexInternalShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btSdfCollisionShape.cpp"]\
+["src/BulletCollision/CollisionShapes/btMiniSDF.cpp"]\
+["src/BulletCollision/CollisionShapes/btUniformScalingShape.cpp"]\
+["src/BulletCollision/Gimpact/btContactProcessing.cpp"]\
+["src/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp"]\
+["src/BulletCollision/Gimpact/btTriangleShapeEx.cpp"]\
+["src/BulletCollision/Gimpact/gim_memory.cpp"]\
+["src/BulletCollision/Gimpact/btGImpactBvh.cpp"]\
+["src/BulletCollision/Gimpact/btGImpactShape.cpp"]\
+["src/BulletCollision/Gimpact/gim_box_set.cpp"]\
+["src/BulletCollision/Gimpact/gim_tri_collision.cpp"]\
+["src/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp"]\
+["src/BulletCollision/Gimpact/btGenericPoolAllocator.cpp"]\
+["src/BulletCollision/Gimpact/gim_contact.cpp"]\
+["src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp"]\
+["src/BulletDynamics/Dynamics/btRigidBody.cpp"]\
+["src/BulletDynamics/Dynamics/btSimulationIslandManagerMt.cpp"]\
+["src/BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.cpp"]\
+["src/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btBatchedConstraints.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btContactConstraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btFixedConstraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btGearConstraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp"]\
+["src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.cpp"]\
+["src/BulletDynamics/MLCPSolvers/btDantzigLCP.cpp"]\
+["src/BulletDynamics/MLCPSolvers/btLemkeAlgorithm.cpp"]\
+["src/BulletDynamics/MLCPSolvers/btMLCPSolver.cpp"]\
+["src/BulletDynamics/Featherstone/btMultiBody.cpp"]\
+["src/BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.cpp"]\
+["src/BulletDynamics/Featherstone/btMultiBodyJointMotor.cpp"]\
+["src/BulletDynamics/Featherstone/btMultiBodyGearConstraint.cpp"]\
+["src/BulletDynamics/Featherstone/btMultiBodyConstraint.cpp"]\
+["src/BulletDynamics/Featherstone/btMultiBodyFixedConstraint.cpp"]\
+["src/BulletDynamics/Featherstone/btMultiBodyPoint2Point.cpp"]\
+["src/BulletDynamics/Featherstone/btMultiBodyConstraintSolver.cpp"]\
+["src/BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.cpp"]\
+["src/BulletDynamics/Featherstone/btMultiBodySliderConstraint.cpp"]\
+["src/BulletDynamics/Vehicle/btRaycastVehicle.cpp"]\
+["src/BulletDynamics/Vehicle/btWheelInfo.cpp"]\
+["src/BulletDynamics/Character/btKinematicCharacterController.cpp"]\
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
+["examples/ThirdPartyLibs/Gwen/Renderers/OpenGL_DebugFont.cpp"]\

if _platform == "linux" or _platform == "linux2":
    libraries = ['dl','pthread']
    CXX_FLAGS += '-D_LINUX '
    CXX_FLAGS += '-DGLEW_STATIC '
    CXX_FLAGS += '-DGLEW_INIT_OPENGL11_FUNCTIONS=1 '
    CXX_FLAGS += '-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1 '
    CXX_FLAGS += '-DDYNAMIC_LOAD_X11_FUNCTIONS '
    CXX_FLAGS += '-DHAS_SOCKLEN_T '
    CXX_FLAGS += '-fno-inline-functions-called-once'
    sources = sources + ["examples/ThirdPartyLibs/enet/unix.c"]\
    +["examples/OpenGLWindow/X11OpenGLWindow.cpp"]\
    +["examples/ThirdPartyLibs/glad/glad.c"]\
    +["examples/ThirdPartyLibs/glad/glad_glx.c"]
    include_dirs += ["examples/ThirdPartyLibs/optionalX11"]
elif _platform == "win32":
    print("win32!")
    libraries = ['Ws2_32','Winmm','User32','Opengl32','kernel32','glu32','Gdi32','Comdlg32']
    CXX_FLAGS += '-DWIN32 '
    CXX_FLAGS += '-DGLEW_STATIC '
    sources = sources + ["examples/ThirdPartyLibs/enet/win32.c"]\
    +["examples/OpenGLWindow/Win32Window.cpp"]\
    +["examples/OpenGLWindow/Win32OpenGLWindow.cpp"]\
    +["examples/ThirdPartyLibs/glad/glad.c"]
elif _platform == "darwin":
    print("darwin!")
    os.environ['LDFLAGS'] = '-framework Cocoa -framework OpenGL'
    CXX_FLAGS += '-DB3_NO_PYTHON_FRAMEWORK '
    CXX_FLAGS += '-DHAS_SOCKLEN_T '
    CXX_FLAGS += '-D_DARWIN '
#    CXX_FLAGS += '-framework Cocoa '
    sources = sources + ["examples/ThirdPartyLibs/enet/unix.c"]\
    +["examples/OpenGLWindow/MacOpenGLWindow.cpp"]\
    +["examples/ThirdPartyLibs/glad/glad.c"]\
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
    +["examples/OpenGLWindow/X11OpenGLWindow.cpp"]\
    +["examples/ThirdPartyLibs/glad/glad.c"]\
    + sources

setup_py_dir = os.path.dirname(os.path.realpath(__file__))

need_files = []
datadir = "examples/pybullet/gym/pybullet_data"

hh = setup_py_dir + "/" + datadir

for root, dirs, files in os.walk(hh):
    for fn in files:
        ext = os.path.splitext(fn)[1][1:]
        if ext and ext in 'yaml index meta data-00000-of-00001 png gif jpg urdf sdf obj mtl dae off stl STL xml '.split():
            fn = root + "/" + fn
            need_files.append(fn[1+len(hh):])

print("found resource files: %i" % len(need_files))
for n in need_files: print("-- %s" % n)
print("packages")
print(find_packages('examples/pybullet/gym'))
print("-----")

setup(
	name = 'pybullet',
	version='2.0.9',
	description='Official Python Interface for the Bullet Physics SDK specialized for Robotics Simulation and Reinforcement Learning',
	long_description='pybullet is an easy to use Python module for physics simulation, robotics and deep reinforcement learning based on the Bullet Physics SDK. With pybullet you can load articulated bodies from URDF, SDF and other file formats. pybullet provides forward dynamics simulation, inverse dynamics computation, forward and inverse kinematics and collision detection and ray intersection queries. Aside from physics simulation, pybullet supports to rendering, with a CPU renderer and OpenGL visualization and support for virtual reality headsets.',
	url='https://github.com/bulletphysics/bullet3',
	author='Erwin Coumans, Yunfei Bai, Jasmine Hsu',
	author_email='erwincoumans@google.com',
	license='zlib',
	platforms='any',
	keywords=['game development', 'virtual reality', 'physics simulation', 'robotics', 'collision detection', 'opengl'],
	ext_modules = [Extension("pybullet", 
	sources =  sources,
	libraries = libraries,
	extra_compile_args=CXX_FLAGS.split(),
	include_dirs = include_dirs + ["src","examples/ThirdPartyLibs","examples/ThirdPartyLibs/glad", "examples/ThirdPartyLibs/enet/include","examples/ThirdPartyLibs/clsocket/src"]
     ) ],
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
