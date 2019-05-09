from __future__ import print_function
import numpy as np


class Obj:

  def __init__(self, fn):
    self.ind_v = 0
    self.ind_vt = 0
    self.ind_vn = 0
    self.fn = fn
    self.out = open(fn + ".tmp", "w")
    self.out.write("mtllib dinnerware.mtl\n")

  def __del__(self):
    self.out.close()
    import shutil
    shutil.move(self.fn + ".tmp", self.fn)

  def push_v(self, v):
    self.out.write("v %f %f %f\n" % (v[0], v[1], v[2]))
    self.ind_v += 1
    return self.ind_v

  def push_vt(self, vt):
    self.out.write("vt %f %f\n" % (vt[0], vt[1]))
    self.ind_vt += 1
    return self.ind_vt

  def push_vn(self, vn):
    vn /= np.linalg.norm(vn)
    self.out.write("vn %f %f %f\n" % (vn[0], vn[1], vn[2]))
    self.ind_vn += 1
    return self.ind_vn


def convex_hull(points, vind, nind, tind, obj):
  "super ineffective"
  cnt = len(points)
  for a in range(cnt):
    for b in range(a + 1, cnt):
      for c in range(b + 1, cnt):
        vec1 = points[a] - points[b]
        vec2 = points[a] - points[c]
        n = np.cross(vec1, vec2)
        n /= np.linalg.norm(n)
        C = np.dot(n, points[a])
        inner = np.inner(n, points)
        pos = (inner <= C + 0.0001).all()
        neg = (inner >= C - 0.0001).all()
        if not pos and not neg: continue
        obj.out.write("f %i//%i %i//%i %i//%i\n" %
                      ((vind[a], nind[a], vind[b], nind[b], vind[c], nind[c]) if
                       (inner - C).sum() < 0 else
                       (vind[a], nind[a], vind[c], nind[c], vind[b], nind[b])))
        #obj.out.write("f %i/%i/%i %i/%i/%i %i/%i/%i\n" % (
        #	(vind[a], tind[a], nind[a], vind[b], tind[b], nind[b], vind[c], tind[c], nind[c])
        #	if (inner - C).sum() < 0 else
        #	(vind[a], tind[a], nind[a], vind[c], tind[c], nind[c], vind[b], tind[b], nind[b]) ) )


def test_convex_hull():
  obj = Obj("convex_test.obj")
  vlist = np.random.uniform(low=-0.1, high=+0.1, size=(100, 3))
  nlist = vlist.copy()
  tlist = np.random.uniform(low=0, high=+1, size=(100, 2))
  vind = [obj.push_v(xyz) for xyz in vlist]
  nind = [obj.push_vn(xyz) for xyz in nlist]
  tind = [obj.push_vt(uv) for uv in tlist]
  convex_hull(vlist, vind, nind, tind, obj)


class Contour:

  def __init__(self):
    self.vprev_vind = None

  def f(self, obj, vlist_vind, vlist_tind, vlist_nind):
    cnt = len(vlist_vind)
    for i1 in range(cnt):
      i2 = i1 - 1
      obj.out.write("f %i/%i/%i %i/%i/%i %i/%i/%i\n" % (
          vlist_vind[i2],
          vlist_tind[i2],
          vlist_nind[i2],
          vlist_vind[i1],
          vlist_tind[i1],
          vlist_nind[i1],
          self.vprev_vind[i1],
          self.vprev_tind[i1],
          self.vprev_nind[i1],
      ))
      obj.out.write("f %i/%i/%i %i/%i/%i %i/%i/%i\n" % (
          vlist_vind[i2],
          vlist_tind[i2],
          vlist_nind[i2],
          self.vprev_vind[i1],
          self.vprev_tind[i1],
          self.vprev_nind[i1],
          self.vprev_vind[i2],
          self.vprev_tind[i2],
          self.vprev_nind[i2],
      ))

  def belt(self, obj, vlist, nlist, tlist):
    vlist_vind = [obj.push_v(xyz) for xyz in vlist]
    vlist_tind = [obj.push_vt(xyz) for xyz in tlist]
    vlist_nind = [obj.push_vn(xyz) for xyz in nlist]
    if self.vprev_vind:
      self.f(obj, vlist_vind, vlist_tind, vlist_nind)
    else:
      self.first_vind = vlist_vind
      self.first_tind = vlist_tind
      self.first_nind = vlist_nind
    self.vprev_vind = vlist_vind
    self.vprev_tind = vlist_tind
    self.vprev_nind = vlist_nind

  def finish(self, obj):
    self.f(obj, self.first_vind, self.first_tind, self.first_nind)


def test_contour():
  RAD1 = 2.0
  RAD2 = 1.5
  obj = Obj("torus.obj")
  obj.out.write("usemtl porcelain\n")
  contour = Contour()
  for step in range(100):
    angle = step / 100.0 * 2 * np.pi
    belt_v = []
    belt_n = []
    belt_t = []
    for b in range(50):
      beta = b / 50.0 * 2 * np.pi
      r = RAD2 * np.cos(beta) + RAD1
      z = RAD2 * np.sin(beta)
      belt_v.append(np.array([np.cos(angle) * r, np.sin(angle) * r, z]))
      belt_n.append(
          np.array([np.cos(angle) * np.cos(beta),
                    np.sin(angle) * np.cos(beta),
                    np.sin(beta)]))
      belt_t.append((0, 0))
    contour.belt(obj, belt_v, belt_n, belt_t)
  contour.finish(obj)


#test_convex_hull()
#test_contour()


class RotationFigureParams:
  pass


def generate_plate(p, obj, collision_prefix):
  contour = Contour()
  belt_vlist_3d_prev = None

  for step in range(p.N_VIZ + 1):
    angle = step / float(p.N_VIZ) * 2 * np.pi

    if step % p.COLLISION_EVERY == 0:
      vlist_3d = []
      for x, y in p.belt_simple:
        vlist_3d.append([np.cos(angle) * x * 1.06, np.sin(angle) * x * 1.06, y])
      if belt_vlist_3d_prev:
        obj2 = Obj(collision_prefix % (step / p.COLLISION_EVERY))
        obj2.out.write("usemtl pan_tefal\n")
        vlist = np.array(vlist_3d + belt_vlist_3d_prev)
        vlist[len(vlist_3d):] *= 1.01  # break points on one plane
        vlist[0, 0:2] += 0.01 * vlist[len(vlist_3d), 0:2]
        vlist[len(vlist_3d), 0:2] += 0.01 * vlist[0, 0:2]
        nlist = np.random.uniform(low=-1, high=+1, size=vlist.shape)
        tlist = np.random.uniform(low=0, high=+1, size=(len(vlist), 2))
        vind = [obj2.push_v(xyz) for xyz in vlist]
        nind = [obj2.push_vn(xyz) for xyz in nlist]
        convex_hull(vlist, vind, nind, None, obj2)
      belt_vlist_3d_prev = vlist_3d
    if step == p.N_VIZ: break

    belt_v = []
    belt_n = []
    belt_t = []
    for x, y, nx, ny in p.belt:
      belt_v.append(np.array([np.cos(angle) * x, np.sin(angle) * x, y]))
      belt_n.append(np.array([np.cos(angle) * nx, np.sin(angle) * nx, ny]))
      if ny - nx >= 0:
        belt_t.append((127.0 / 512 + np.cos(angle) * x / p.RAD_HIGH * 105 / 512,
                       (512 - 135.0) / 512 + np.sin(angle) * x / p.RAD_HIGH * 105 / 512))
      else:
        belt_t.append((382.0 / 512 + np.cos(angle) * x / p.RAD_HIGH * 125 / 512,
                       (512 - 380.0) / 512 + np.sin(angle) * x / p.RAD_HIGH * 125 / 512))
    contour.belt(obj, belt_v, belt_n, belt_t)

  contour.finish(obj)


def tefal():
  p = RotationFigureParams()
  p.RAD_LOW = 0.240 / 2
  p.RAD_HIGH = 0.255 / 2
  p.H = 0.075
  p.THICK = 0.005
  p.N_VIZ = 30
  p.COLLISION_EVERY = 5
  p.belt = [
      (p.RAD_HIGH - p.THICK, p.H, -1, 0),  # x y norm
      (p.RAD_HIGH, p.H, 0, 1),
      (p.RAD_HIGH + p.THICK, p.H, +1, 0),
      (p.RAD_LOW + p.THICK, p.THICK, +1, 0),
      (p.RAD_LOW, 0, 0, -1),
      (0, 0, 0, -1),
      (0, p.THICK, 0, 1),
      (p.RAD_LOW - p.THICK, p.THICK, 0, 1),
      (p.RAD_LOW - p.THICK, 3 * p.THICK, -1, 0),
  ]
  p.belt.reverse()
  p.belt_simple = [(p.RAD_HIGH - p.THICK, p.H), (p.RAD_HIGH + p.THICK, p.H), (p.RAD_LOW, 0),
                   (p.RAD_LOW - p.THICK, 0)]
  obj = Obj("pan_tefal.obj")
  obj.out.write("usemtl pan_tefal\n")
  generate_plate(p, obj, "pan_tefal-collision%02i.obj")


def plate():
  p = RotationFigureParams()
  p.RAD_LOW = 0.110 / 2
  p.RAD_HIGH = 0.190 / 2
  p.H = 0.060
  p.THICK = 0.003
  p.N_VIZ = 30
  p.COLLISION_EVERY = 5
  p.belt = [
      (p.RAD_HIGH - p.THICK, p.H, -0.9, 0.5),  # x y norm
      (p.RAD_HIGH, p.H, 0, 1),
      (p.RAD_HIGH + p.THICK, p.H, +1, 0),
      (p.RAD_LOW + p.THICK, p.THICK, +1, 0),
      (p.RAD_LOW, 0, 0, -1),
      (0, 0, 0, -1),
      (0, p.THICK, 0, 1),
      (p.RAD_LOW - 3 * p.THICK, p.THICK, 0, 1),
      (p.RAD_LOW - p.THICK, 3 * p.THICK, -0.5, 1.0),
  ]
  p.belt.reverse()
  p.belt_simple = [(p.RAD_HIGH - p.THICK, p.H), (p.RAD_HIGH + p.THICK, p.H), (p.RAD_LOW, 0),
                   (p.RAD_LOW - p.THICK, 0)]
  obj = Obj("plate.obj")
  obj.out.write("usemtl solid_color\n")
  generate_plate(p, obj, "plate-collision%02i.obj")


plate()
