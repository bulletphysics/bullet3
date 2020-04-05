//
//  btModifiedGramSchmidt.h
//  LinearMath
//
//  Created by Xuchen Han on 4/4/20.
//

#ifndef btModifiedGramSchmidt_h
#define btModifiedGramSchmidt_h

#include "btReducedVector.h"
#include "btAlignedObjectArray.h"

template<class TV>
class btModifiedGramSchmidt
{
public:
    btAlignedObjectArray<TV> m_in;
    btAlignedObjectArray<TV> m_out;
    
    btModifiedGramSchmidt(const btAlignedObjectArray<TV>& vecs): m_in(vecs)
    {
        m_out.resize(0);
    }
    
    void solve()
    {
        m_out.resize(m_in.size());
        for (int i = 0; i < m_in.size(); ++i)
        {
            TV v(m_in[i]);
            v.print();
            for (int j = 0; j < i; ++j)
            {
                v = v - v.proj(m_out[j]);
                v.print();
            }
            v.normalize();
            v.print();
            m_out[i] = v;
            printf("===========\n");
        }
    }
    
    void test()
    {
        btScalar eps = SIMD_EPSILON;
        for (int i = 0; i < m_out.size(); ++i)
        {
            for (int j = 0; j < m_out.size(); ++j)
            {
                if (i == j)
                {
                    if (std::abs(1-m_out[i].dot(m_out[j])) > eps && std::abs(m_out[i].dot(m_out[j])) > eps)
                    {
                        printf("vec[%d] is not unit, norm squared = %f\n", i,m_out[i].dot(m_out[j]));
                    }
                }
                else
                {
                    if (std::abs(m_out[i].dot(m_out[j])) > eps)
                    {
                        printf("vec[%d] and vec[%d] is not orthogonal, dot product = %f\n", i, j, m_out[i].dot(m_out[j]));
                    }
                }
            }
        }
    }
};
template class btModifiedGramSchmidt<btReducedVector>;
#endif /* btModifiedGramSchmidt_h */
