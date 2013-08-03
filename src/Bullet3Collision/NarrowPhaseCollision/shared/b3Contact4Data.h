#ifndef B3_CONTACT4DATA_H
#define B3_CONTACT4DATA_H

#include "Bullet3Common/shared/b3Float4.h"

typedef struct
{
	b3Float4	m_worldPos[4];
	b3Float4	m_worldNormal;	//	w: m_nPoints
	unsigned int  m_coeffs;
	unsigned int m_batchIdx;

	int m_bodyAPtrAndSignBit;//x:m_bodyAPtr, y:m_bodyBPtr
	int m_bodyBPtrAndSignBit;

	int	m_childIndexA;
	int	m_childIndexB;
	int m_unused1;
	int m_unused2;

} b3Contact4Data;

#endif //B3_CONTACT4DATA_H