import pybullet as p
useEGL = False
useEGLGUI = False

if useEGL:
  if useEGLGUI:
    p.connect(p.GUI, "window_backend=2")
  else:
    p.connect(p.DIRECT)
    p.loadPlugin("eglRendererPlugin")
else:
  p.connect(p.GUI)

p.loadURDF("threecubes.urdf", flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
while (1):

  viewmat = [
      0.642787516117096, -0.4393851161003113, 0.6275069713592529, 0.0, 0.766044557094574,
      0.36868777871131897, -0.5265407562255859, 0.0, -0.0, 0.8191521167755127, 0.5735764503479004,
      0.0, 2.384185791015625e-07, 2.384185791015625e-07, -5.000000476837158, 1.0
  ]
  projmat = [
      0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0,
      0.0, 0.0, -0.02000020071864128, 0.0
  ]

  p.getCameraImage(64,
                   64,
                   viewMatrix=viewmat,
                   projectionMatrix=projmat,
                   flags=p.ER_NO_SEGMENTATION_MASK)
  p.stepSimulation()
