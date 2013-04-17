#include "btPolarDecomposition.h"
#include "BulletCommon/b3MinMax.h"

namespace
{
  b3Scalar abs_column_sum(const b3Matrix3x3& a, int i)
  {
    return btFabs(a[0][i]) + btFabs(a[1][i]) + btFabs(a[2][i]);
  }

  b3Scalar abs_row_sum(const b3Matrix3x3& a, int i)
  {
    return btFabs(a[i][0]) + btFabs(a[i][1]) + btFabs(a[i][2]);
  }

  b3Scalar p1_norm(const b3Matrix3x3& a)
  {
    const b3Scalar sum0 = abs_column_sum(a,0);
    const b3Scalar sum1 = abs_column_sum(a,1);
    const b3Scalar sum2 = abs_column_sum(a,2);
    return btMax(btMax(sum0, sum1), sum2);
  }

  b3Scalar pinf_norm(const b3Matrix3x3& a)
  {
    const b3Scalar sum0 = abs_row_sum(a,0);
    const b3Scalar sum1 = abs_row_sum(a,1);
    const b3Scalar sum2 = abs_row_sum(a,2);
    return btMax(btMax(sum0, sum1), sum2);
  }
}

const b3Scalar btPolarDecomposition::DEFAULT_TOLERANCE = b3Scalar(0.0001);
const unsigned int btPolarDecomposition::DEFAULT_MAX_ITERATIONS = 16;

btPolarDecomposition::btPolarDecomposition(b3Scalar tolerance, unsigned int maxIterations)
: m_tolerance(tolerance)
, m_maxIterations(maxIterations)
{
}

unsigned int btPolarDecomposition::decompose(const b3Matrix3x3& a, b3Matrix3x3& u, b3Matrix3x3& h) const
{
  // Use the 'u' and 'h' matrices for intermediate calculations
  u = a;
  h = a.inverse();

  for (unsigned int i = 0; i < m_maxIterations; ++i)
  {
    const b3Scalar h_1 = p1_norm(h);
    const b3Scalar h_inf = pinf_norm(h);
    const b3Scalar u_1 = p1_norm(u);
    const b3Scalar u_inf = pinf_norm(u);

    const b3Scalar h_norm = h_1 * h_inf;
    const b3Scalar u_norm = u_1 * u_inf;

    // The matrix is effectively singular so we cannot invert it
    if (btFuzzyZero(h_norm) || btFuzzyZero(u_norm))
      break;

    const b3Scalar gamma = btPow(h_norm / u_norm, 0.25f);
    const b3Scalar inv_gamma = 1.0 / gamma;

    // Determine the delta to 'u'
    const b3Matrix3x3 delta = (u * (gamma - 2.0) + h.transpose() * inv_gamma) * 0.5;

    // Update the matrices
    u += delta;
    h = u.inverse();

    // Check for convergence
    if (p1_norm(delta) <= m_tolerance * u_1)
    {
      h = u.transpose() * a;
      h = (h + h.transpose()) * 0.5;
      return i;
    }
  }

  // The algorithm has failed to converge to the specified tolerance, but we
  // want to make sure that the matrices returned are in the right form.
  h = u.transpose() * a;
  h = (h + h.transpose()) * 0.5;

  return m_maxIterations;
}

unsigned int btPolarDecomposition::maxIterations() const
{
  return m_maxIterations;
}

unsigned int polarDecompose(const b3Matrix3x3& a, b3Matrix3x3& u, b3Matrix3x3& h)
{
  static btPolarDecomposition polar;
  return polar.decompose(a, u, h);
}

