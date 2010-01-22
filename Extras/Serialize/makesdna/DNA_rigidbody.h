
#ifndef DNA_RIGIDBODY_H
#define DNA_RIGIDBODY_H


struct	PointerArray
{
	int		m_size;
	int		m_capacity;
	void	*m_data;
};


struct btPhysicsSystem
{
	PointerArray	m_collisionShapes;
	PointerArray	m_collisionObjects;
	PointerArray	m_constraints;
};

///we need this to compute the pointer sizes
struct ListBase
{
	void *first;
	void *last;
};


#endif
