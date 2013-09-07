#ifndef CPU_SOFTCLOTH_INTERNAL_DATA_H
#define CPU_SOFTCLOTH_INTERNAL_DATA_H

#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3Common/b3Vector3.h"

struct ClothSpring
{
	int		m_particleIndexA;
	int		m_particleIndexB;
	float	m_restLength;
	int		m_material;
};

struct ClothMaterial
{
	float	m_stiffness;
	float	m_damping;
};

struct  CpuSoftClothDemoInternalData
{
	b3AlignedObjectArray<ClothSpring> m_springs;
	b3AlignedObjectArray<ClothMaterial>	m_materials;
	b3AlignedObjectArray<b3Vector3>	m_velocities;
	b3AlignedObjectArray<b3Vector3>	m_forces;
	b3AlignedObjectArray<float>	m_particleMasses;	
};

#endif //CPU_SOFTCLOTH_INTERNAL_DATA_H

