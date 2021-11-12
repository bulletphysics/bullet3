//#define EIGEN_POWER_USE_PREFETCH  // Use prefetching in gemm routines
#ifdef EIGEN_POWER_USE_PREFETCH
#define EIGEN_POWER_PREFETCH(p)  prefetch(p)
#else
#define EIGEN_POWER_PREFETCH(p)
#endif

namespace Eigen {

namespace internal {

template<typename Scalar, typename Packet, typename DataMapper, typename Index, const Index accRows, const Index accCols>
EIGEN_ALWAYS_INLINE void gemm_extra_row(
  const DataMapper& res,
  const Scalar* lhs_base,
  const Scalar* rhs_base,
  Index depth,
  Index strideA,
  Index offsetA,
  Index row,
  Index col,
  Index rows,
  Index cols,
  Index remaining_rows,
  const Packet& pAlpha,
  const Packet& pMask);

template<typename Scalar, typename Packet, typename DataMapper, typename Index, const Index accCols, bool ConjugateLhs, bool ConjugateRhs, bool LhsIsReal, bool RhsIsReal>
EIGEN_STRONG_INLINE void gemm_extra_cols(
  const DataMapper& res,
  const Scalar* blockA,
  const Scalar* blockB,
  Index depth,
  Index strideA,
  Index offsetA,
  Index strideB,
  Index offsetB,
  Index col,
  Index rows,
  Index cols,
  Index remaining_rows,
  const Packet& pAlpha,
  const Packet& pMask);

template<typename Packet>
EIGEN_ALWAYS_INLINE Packet bmask(const int remaining_rows);

template<typename Scalar, typename Packet, typename Packetc, typename DataMapper, typename Index, const Index accRows, const Index accCols, bool ConjugateLhs, bool ConjugateRhs, bool LhsIsReal, bool RhsIsReal>
EIGEN_ALWAYS_INLINE void gemm_complex_extra_row(
  const DataMapper& res,
  const Scalar* lhs_base,
  const Scalar* rhs_base,
  Index depth,
  Index strideA,
  Index offsetA,
  Index strideB,
  Index row,
  Index col,
  Index rows,
  Index cols,
  Index remaining_rows,
  const Packet& pAlphaReal,
  const Packet& pAlphaImag,
  const Packet& pMask);

template<typename Scalar, typename Packet, typename Packetc, typename DataMapper, typename Index, const Index accCols, bool ConjugateLhs, bool ConjugateRhs, bool LhsIsReal, bool RhsIsReal>
EIGEN_STRONG_INLINE void gemm_complex_extra_cols(
  const DataMapper& res,
  const Scalar* blockA,
  const Scalar* blockB,
  Index depth,
  Index strideA,
  Index offsetA,
  Index strideB,
  Index offsetB,
  Index col,
  Index rows,
  Index cols,
  Index remaining_rows,
  const Packet& pAlphaReal,
  const Packet& pAlphaImag,
  const Packet& pMask);

template<typename Scalar, typename Packet>
EIGEN_ALWAYS_INLINE Packet ploadLhs(const Scalar* lhs);

template<typename DataMapper, typename Packet, typename Index, const Index accCols, int StorageOrder, bool Complex, int N>
EIGEN_ALWAYS_INLINE void bload(PacketBlock<Packet,N>& acc, const DataMapper& res, Index row, Index col);

template<typename Packet, int N>
EIGEN_ALWAYS_INLINE void bscale(PacketBlock<Packet,N>& acc, PacketBlock<Packet,N>& accZ, const Packet& pAlpha);

template<typename Packet, int N>
EIGEN_ALWAYS_INLINE void bscalec(PacketBlock<Packet,N>& aReal, PacketBlock<Packet,N>& aImag, const Packet& bReal, const Packet& bImag, PacketBlock<Packet,N>& cReal, PacketBlock<Packet,N>& cImag);

// Grab two decouples real/imaginary PacketBlocks and return two coupled (real/imaginary pairs) PacketBlocks.
template<typename Packet, typename Packetc, int N>
EIGEN_ALWAYS_INLINE void bcouple_common(PacketBlock<Packet,N>& taccReal, PacketBlock<Packet,N>& taccImag, PacketBlock<Packetc, N>& acc1, PacketBlock<Packetc, N>& acc2)
{
  acc1.packet[0].v = vec_mergeh(taccReal.packet[0], taccImag.packet[0]);
  if (N > 1) {
    acc1.packet[1].v = vec_mergeh(taccReal.packet[1], taccImag.packet[1]);
  }
  if (N > 2) {
    acc1.packet[2].v = vec_mergeh(taccReal.packet[2], taccImag.packet[2]);
  }
  if (N > 3) {
    acc1.packet[3].v = vec_mergeh(taccReal.packet[3], taccImag.packet[3]);
  }

  acc2.packet[0].v = vec_mergel(taccReal.packet[0], taccImag.packet[0]);
  if (N > 1) {
    acc2.packet[1].v = vec_mergel(taccReal.packet[1], taccImag.packet[1]);
  }
  if (N > 2) {
    acc2.packet[2].v = vec_mergel(taccReal.packet[2], taccImag.packet[2]);
  }
  if (N > 3) {
    acc2.packet[3].v = vec_mergel(taccReal.packet[3], taccImag.packet[3]);
  }
}

template<typename Packet, typename Packetc, int N>
EIGEN_ALWAYS_INLINE void bcouple(PacketBlock<Packet,N>& taccReal, PacketBlock<Packet,N>& taccImag, PacketBlock<Packetc,N*2>& tRes, PacketBlock<Packetc, N>& acc1, PacketBlock<Packetc, N>& acc2)
{
  bcouple_common<Packet, Packetc, N>(taccReal, taccImag, acc1, acc2);

  acc1.packet[0] = padd<Packetc>(tRes.packet[0], acc1.packet[0]);
  if (N > 1) {
    acc1.packet[1] = padd<Packetc>(tRes.packet[1], acc1.packet[1]);
  }
  if (N > 2) {
    acc1.packet[2] = padd<Packetc>(tRes.packet[2], acc1.packet[2]);
  }
  if (N > 3) {
    acc1.packet[3] = padd<Packetc>(tRes.packet[3], acc1.packet[3]);
  }

  acc2.packet[0] = padd<Packetc>(tRes.packet[0+N], acc2.packet[0]);
  if (N > 1) {
    acc2.packet[1] = padd<Packetc>(tRes.packet[1+N], acc2.packet[1]);
  }
  if (N > 2) {
    acc2.packet[2] = padd<Packetc>(tRes.packet[2+N], acc2.packet[2]);
  }
  if (N > 3) {
    acc2.packet[3] = padd<Packetc>(tRes.packet[3+N], acc2.packet[3]);
  }
}

// This is necessary because ploadRhs for double returns a pair of vectors when MMA is enabled.
template<typename Scalar, typename Packet>
EIGEN_ALWAYS_INLINE Packet ploadRhs(const Scalar* rhs)
{
  return ploadu<Packet>(rhs);
}

} // end namespace internal
} // end namespace Eigen
