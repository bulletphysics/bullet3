def dot(A, b):
  """Dot product between a 2D matrix and a 1D vector"""
  return [sum([aij * bj for aij, bj in zip(ai, b)]) for ai in A]


def allclose(a, b, tol=1e-7):
  """Are all elements of a vector close to one another"""
  return all([abs(ai - bi) < tol for ai, bi in zip(a, b)])
