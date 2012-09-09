#ifndef BTCHOLESKYDECOMPOSITION_H
#define BTCHOLESKYDECOMPOSITION_H

#include "LinearMath/btMatrix3x3.h"

struct btCholeskyDecomposition
{
  enum Result
  {
    SUCCESS,
    FAILURE_SYMMETRY,
    FAILURE_POSITIVE_DEFINITE
  };
};

int choleskyDecompose(const btMatrix3x3& A, btMatrix3x3& L);

#endif // BTCHOLESKYDECOMPOSITION_H

