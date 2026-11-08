"""Regression tests for getCameraImage lighting with the EGL renderer.

Run with a built eglRenderer extension available on PYTHONPATH, or set
PYBULLET_EGL_PLUGIN to the absolute path of the plugin under test.
Requires an EGL-capable OpenGL device and NumPy; uses no external assets.
"""
import importlib.util
import os
import unittest

import numpy as np
import pybullet as p


class TestEglLighting(unittest.TestCase):
  flags = 0

  @classmethod
  def setUpClass(cls):
    cls.client = p.connect(p.DIRECT)
    plugin = os.environ.get('PYBULLET_EGL_PLUGIN')
    if plugin is None:
      spec = importlib.util.find_spec('eglRenderer')
      if spec is None:
        p.disconnect(cls.client)
        raise unittest.SkipTest('eglRenderer extension is not installed')
      plugin = spec.origin
    if p.loadPlugin(plugin, '_eglRendererPlugin', physicsClientId=cls.client) < 0:
      p.disconnect(cls.client)
      raise RuntimeError('Cannot load the EGL renderer')
    cls.view = p.computeViewMatrix([2, 0, 2], [0, 0, 0], [0, 0, 1])
    cls.projection = p.computeProjectionMatrixFOV(45, 1, 0.1, 10)

  @classmethod
  def tearDownClass(cls):
    p.disconnect(cls.client)

  def setUp(self):
    p.resetSimulation(physicsClientId=self.client)
    visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.6, rgbaColor=[1, 1, 1, 1],
                                specularColor=[1, 1, 1], physicsClientId=self.client)
    self.body = p.createMultiBody(baseVisualShapeIndex=visual, physicsClientId=self.client)

  def render(self, **overrides):
    settings = dict(lightDirection=[2, 0, 2], lightColor=[1, 1, 1],
                    lightAmbientCoeff=0, lightDiffuseCoeff=0, lightSpecularCoeff=0)
    settings.update(overrides)
    _, _, rgba, _, segmentation = p.getCameraImage(
        128, 128, self.view, self.projection, renderer=p.ER_BULLET_HARDWARE_OPENGL,
        flags=self.flags, physicsClientId=self.client, **settings)
    rgb = np.asarray(rgba, dtype=np.uint8).reshape(128, 128, 4)[..., :3]
    mask = np.asarray(segmentation).reshape(128, 128) == self.body
    self.assertGreater(np.count_nonzero(mask), 100, 'Test sphere is not visible')
    return rgb[mask].astype(float)

  def test_zero_coefficients_disable_lighting(self):
    self.assertLessEqual(self.render().max(), 1)

  def test_ambient_coefficient(self):
    dark = self.render(lightAmbientCoeff=0.2)
    bright = self.render(lightAmbientCoeff=0.8)
    self.assertGreater(bright.mean(), dark.mean() + 100)
    self.assertAlmostEqual(dark.mean(), 51, delta=2)
    self.assertAlmostEqual(bright.mean(), 204, delta=2)

  def test_diffuse_coefficient(self):
    dark = self.render(lightDiffuseCoeff=0)
    bright = self.render(lightDiffuseCoeff=1)
    self.assertGreater(bright.mean(), dark.mean() + 20)

  def test_specular_coefficient(self):
    dark = self.render(lightSpecularCoeff=0)
    bright = self.render(lightSpecularCoeff=1)
    self.assertGreater(bright.max(), dark.max() + 20)
    self.assertGreater(np.count_nonzero(bright > dark + 10), 10)

  def test_light_color(self):
    red = self.render(lightDiffuseCoeff=1, lightColor=[1, 0, 0]).mean(axis=0)
    blue = self.render(lightDiffuseCoeff=1, lightColor=[0, 0, 1]).mean(axis=0)
    self.assertGreater(red[0], red[2] + 20)
    self.assertGreater(blue[2], blue[0] + 20)

  def test_ambient_is_independent_of_light_color(self):
    red = self.render(lightAmbientCoeff=0.4, lightColor=[1, 0, 0])
    blue = self.render(lightAmbientCoeff=0.4, lightColor=[0, 0, 1])
    np.testing.assert_allclose(red, blue, atol=1)
    self.assertAlmostEqual(red.mean(), 102, delta=2)


class TestEglProjectiveLighting(TestEglLighting):
  flags = p.ER_USE_PROJECTIVE_TEXTURE


if __name__ == '__main__':
  unittest.main(verbosity=2)
