import numpy as np

from quaternion import qrot, qinverse


def normalize_screen_coordinates(X, w, h):
  assert X.shape[-1] == 2
  # Normalize so that [0, w] is mapped to [-1, 1], while preserving the aspect ratio
  return X / w * 2 - [1, h / w]


def image_coordinates(X, w, h):
  assert X.shape[-1] == 2
  # Reverse camera frame normalization
  return (X + [1, h / w]) * w / 2


def world_to_camera(X, R, t):
  Rt = qinverse(R)
  Q = np.tile(Rt, (*X.shape[:-1], 1))
  V = X - t
  return qrot(Q, V)


def camera_to_world(X, R, t):
  Q = np.tile(R, (*X.shape[:-1], 1))
  V = X
  return qrot(Q, V) + t
