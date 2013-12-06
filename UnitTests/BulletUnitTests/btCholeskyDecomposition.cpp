#include "btCholeskyDecomposition.h"

int choleskyDecompose(const btMatrix3x3& A, btMatrix3x3& L)
{
  // TODO Check that the A matrix is symmetric

  // Iterate over the elements of the upper triangle of A
  for (unsigned int i = 0; i < 3; ++i)
  {
    for (unsigned int j = i; j < 3; ++j)
    {
      btScalar sum = A[i][j];
      for (unsigned int k = 0; k < i; ++k)
      {
        sum -= L[i][k] * L[j][k];
      }

      if (i != j)
      {
        L[j][i] = sum / L[i][i];
      }
      else
      {
        if (sum <= btScalar(0))
        {
          return btCholeskyDecomposition::FAILURE_POSITIVE_DEFINITE;
        }

        L[i][i] = sqrt(sum);
      }
    }
  }

  L[0][1] = btScalar(0);
  L[0][2] = btScalar(0);
  L[1][2] = btScalar(0);

  return btCholeskyDecomposition::SUCCESS;
}

