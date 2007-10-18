#ifndef COLLISION_PROCESSING_H
#define COLLISION_PROCESSING_H

#include "../FrameWork/Test.h"

class CollisionProcessing : public Test
{
public:
void Step(Settings* settings)
{

}
static Test* Create()
{
return new CollisionProcessing;
}
};

#endif //

