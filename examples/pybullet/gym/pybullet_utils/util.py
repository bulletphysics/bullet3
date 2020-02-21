import random
import numpy as np


def set_global_seeds(seed):
  try:
    import tf.compat.v1 as tf
  except ImportError:
    pass
  else:
    tf.set_random_seed(seed)
  np.random.seed(seed)
  random.seed(seed)
  return
