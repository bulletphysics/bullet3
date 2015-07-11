/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2014 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///Original author: Erwin Coumans, October 2014
///Initial version of this low-level GJK/EPA/MPR convex-convex collision test
///You can provide your own support function in combination with the template functions
///See btComputeGjkEpaSphereSphereCollision below for an example
///Todo: the test needs proper coverage and using a convex hull point cloud
///Also the GJK, EPA and MPR should be improved, both quality and performance


#include <gtest/gtest.h>


#include "SphereSphereCollision.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"



#include "BulletCollision/NarrowPhaseCollision/btComputeGjkEpaPenetration.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa3.h"
#include "BulletCollision/NarrowPhaseCollision/btMprPenetration.h"



btVector3 MyBulletShapeSupportFunc(const void* shapeAptr, const btVector3& dir, bool includeMargin)
{
    btConvexShape* shape = (btConvexShape*) shapeAptr;
    if (includeMargin)
    {
        return shape->localGetSupportingVertex(dir);
    }
    
    return shape->localGetSupportingVertexWithoutMargin(dir);
}

btVector3 MyBulletShapeCenterFunc(const void* shapeAptr)
{
    return btVector3(0,0,0);
}

enum SphereSphereTestMethod
{
    SSTM_ANALYTIC,
    SSTM_GJKEPA,
    SSTM_GJKEPA_RADIUS_NOT_FULL_MARGIN,
    SSTM_GJKMPR
};



struct ConvexWrap
{
    btConvexShape* m_convex;
    btTransform m_worldTrans;
    inline btScalar getMargin() const
    {
        return m_convex->getMargin();
    }
    inline btVector3 getObjectCenterInWorld() const
    {
        return m_worldTrans.getOrigin();
    }
    inline const btTransform& getWorldTransform() const
    {
        return m_worldTrans;
    }
    inline btVector3 getLocalSupportWithMargin(const btVector3& dir) const
    {
        return m_convex->localGetSupportingVertex(dir);
    }
    inline btVector3 getLocalSupportWithoutMargin(const btVector3& dir) const
    {
        return m_convex->localGetSupportingVertexWithoutMargin(dir);
    }
};

inline int btComputeGjkEpaSphereSphereCollision(const btSphereSphereCollisionDescription& input, btDistanceInfo* distInfo,SphereSphereTestMethod method)
{
    ///for spheres it is best to use a 'point' and set the margin to the radius (which is what btSphereShape does)
    btSphereShape singleSphereA(input.m_radiusA);
    btSphereShape singleSphereB(input.m_radiusB);
    btVector3 org(0,0,0);
    btScalar radA =input.m_radiusA;
    btScalar radB =input.m_radiusB;

    ConvexWrap a,b;
    a.m_worldTrans = input.m_sphereTransformA;
    b.m_worldTrans = input.m_sphereTransformB;;
    
    btMultiSphereShape multiSphereA(&org,&radA,1);
    btMultiSphereShape multiSphereB(&org,&radB,1);
    
    btGjkCollisionDescription colDesc;
    switch (method)
    {
        case SSTM_GJKEPA_RADIUS_NOT_FULL_MARGIN:
        {
           
            a.m_convex = &multiSphereA;
            b.m_convex = &multiSphereB;
            break;
       }
        default:
        {
            a.m_convex = &singleSphereA;
            b.m_convex = &singleSphereB;
        }
    };
    
    btVoronoiSimplexSolver simplexSolver;
    simplexSolver.reset();
    
    int res=-1;
    ///todo(erwincoumans): improve convex-convex quality and performance
    ///also compare with https://code.google.com/p/bullet/source/browse/branches/PhysicsEffects/src/base_level/collision/pfx_gjk_solver.cpp
    switch (method)
    {
        case SSTM_GJKEPA_RADIUS_NOT_FULL_MARGIN:
        case SSTM_GJKEPA:
        {
            res = btComputeGjkEpaPenetration(a,b,colDesc,simplexSolver, distInfo);
            break;
        }
        case SSTM_GJKMPR:
        {
            res = btComputeGjkDistance(a,b,colDesc,distInfo);
            if (res==0)
            {
             //   printf("use GJK results in distance %f\n",distInfo->m_distance);
                return res;
            } else
            {
                btMprCollisionDescription mprDesc;
                res = btComputeMprPenetration(a,b,mprDesc, distInfo);

//                if (res==0)
//                {
//                    printf("use MPR results in distance %f\n",distInfo->m_distance);
//                }
            }
            break;
        }
        default:
        {
            
            btAssert(0);
        }
    }
    return res;
}



void testSphereSphereDistance(SphereSphereTestMethod method, btScalar abs_error)
{
    {
        btSphereSphereCollisionDescription ssd;
        ssd.m_sphereTransformA.setIdentity();
        ssd.m_sphereTransformB.setIdentity();
        ssd.m_radiusA = 0.f;
        ssd.m_radiusB = 0.f;
        btDistanceInfo distInfo;
        int result = btComputeSphereSphereCollision(ssd,&distInfo);
        ASSERT_EQ(0,result);
        ASSERT_EQ(btScalar(0), distInfo.m_distance);
    }
    
    for (int rb=1;rb<10;rb++)
        for (int z=-20;z<20;z++)
        {
            for (int j=1;j<10;j++)
            {
                for (int i=-20;i<20;i++)
                {
                    if (i!=z)//skip co-centric spheres for now (todo(erwincoumans) fix this)
                    {
                        btSphereSphereCollisionDescription ssd;
                        ssd.m_sphereTransformA.setIdentity();
                        ssd.m_sphereTransformA.setOrigin(btVector3(0,btScalar(i),0));
                        ssd.m_sphereTransformB.setIdentity();
                        ssd.m_sphereTransformB.setOrigin(btVector3(0,btScalar(z),0));
                        ssd.m_radiusA = btScalar(j);
                        ssd.m_radiusB = btScalar(rb)*btScalar(0.1);
                        btDistanceInfo distInfo;
                        int result=-1;
                        switch (method)
                        {
                            case SSTM_ANALYTIC:
                            {
                                result = btComputeSphereSphereCollision(ssd,&distInfo);
                                break;
                            }
                            case SSTM_GJKMPR:
                            case SSTM_GJKEPA:
                            case SSTM_GJKEPA_RADIUS_NOT_FULL_MARGIN:
                            {
                                 result = btComputeGjkEpaSphereSphereCollision(ssd,&distInfo, method);
                                break;
                            }
                            default:
                            {
                                ASSERT_EQ(0,1);
                                btAssert(0);
                                break;
                            }
                        }
                        //  int result = btComputeSphereSphereCollision(ssd,&distInfo);
#if 0
                        printf("sphereA(pos=[%f,%f,%f],r=%f)-sphereB(pos=[%f,%f,%f],r=%f) Dist=%f,normalOnB[%f,%f,%f],pA=[%f,%f,%f],pB[%f,%f,%f]\n",
                               ssd.m_sphereTransformA.getOrigin()[0],ssd.m_sphereTransformA.getOrigin()[1],ssd.m_sphereTransformA.getOrigin()[2],ssd.m_radiusA,
                               ssd.m_sphereTransformB.getOrigin()[0],ssd.m_sphereTransformB.getOrigin()[1],ssd.m_sphereTransformB.getOrigin()[2],ssd.m_radiusB,
                               distInfo.m_distance,distInfo.m_normalBtoA[0],distInfo.m_normalBtoA[1],distInfo.m_normalBtoA[2],
                               distInfo.m_pointOnA[0],distInfo.m_pointOnA[1],distInfo.m_pointOnA[2],
                               distInfo.m_pointOnB[0],distInfo.m_pointOnB[1],distInfo.m_pointOnB[2]);
#endif
                        ASSERT_EQ(0,result);
                        ASSERT_NEAR(btFabs(btScalar(i-z))-btScalar(j)-ssd.m_radiusB, distInfo.m_distance, abs_error);
                        btVector3 computedA = distInfo.m_pointOnB+distInfo.m_distance*distInfo.m_normalBtoA;
                        ASSERT_NEAR(computedA.x(),distInfo.m_pointOnA.x(),abs_error);
                        ASSERT_NEAR(computedA.y(),distInfo.m_pointOnA.y(),abs_error);
                        ASSERT_NEAR(computedA.z(),distInfo.m_pointOnA.z(),abs_error);
                    }
                }
            }
        }

}

TEST(BulletCollisionTest, GjkMPRSphereSphereDistance) {
    testSphereSphereDistance(SSTM_GJKMPR, 0.0001);
}


TEST(BulletCollisionTest, GjkEpaSphereSphereDistance) {
    testSphereSphereDistance(SSTM_GJKEPA, 0.00001);
}

TEST(BulletCollisionTest, GjkEpaSphereSphereRadiusNotFullMarginDistance) {
    testSphereSphereDistance(SSTM_GJKEPA_RADIUS_NOT_FULL_MARGIN,  0.1);
}

TEST(BulletCollisionTest, AnalyticSphereSphereDistance) {
 testSphereSphereDistance(SSTM_ANALYTIC, 0.00001);
}





int main(int argc, char **argv) {
#if _MSC_VER
        _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
        //void *testWhetherMemoryLeakDetectionWorks = malloc(1);
#endif
        ::testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}
