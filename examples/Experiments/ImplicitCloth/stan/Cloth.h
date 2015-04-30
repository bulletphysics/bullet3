#ifndef STAN_CLOTH_H
#define STAN_CLOTH_H

#include "SpringNetwork.h"

class Cloth : public SpringNetwork
{
 public:
       int w,h;
       
       float3 color; // for debug rendering
       Cloth(const char* _name,int _n);
       ~Cloth();
};

Cloth *ClothCreate(int w,int h,float size);

#endif //STAN_CLOTH_H
