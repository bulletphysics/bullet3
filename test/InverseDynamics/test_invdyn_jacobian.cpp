// Test of kinematic consistency: check if finite differences of velocities, accelerations
// match positions

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <gtest/gtest.h>

#include "Bullet3Common/b3Random.h"

#include "CloneTreeCreator.hpp"
#include "CoilCreator.hpp"
#include "DillCreator.hpp"
#include "RandomTreeCreator.hpp"
#include "BulletInverseDynamics/MultiBodyTree.hpp"
#include "MultiBodyTreeDebugGraph.hpp"

using namespace btInverseDynamics;

#if (defined BT_ID_HAVE_MAT3X) && (defined BT_ID_WITH_JACOBIANS)
// minimal smart pointer to make this work for c++2003
template <typename T>
class ptr {
    ptr();
    ptr(const ptr&);
public:
    ptr(T* p) : m_p(p) {};
    ~ptr() { delete m_p; }
    T& operator*() { return *m_p; }
    T* operator->() { return m_p; }
    T*get() {return m_p;}
    const T*get() const {return m_p;}
    friend bool operator==(const ptr<T>& lhs, const ptr<T>& rhs) { return rhs.m_p == lhs.m_p; }
    friend bool operator!=(const ptr<T>& lhs, const ptr<T>& rhs) { return !(rhs.m_p == lhs.m_p);
}

private:
    T* m_p;
};

void calculateDotJacUError(const MultiBodyTreeCreator& creator, const int nloops,
                           double* max_error) {
    // tree1 is used as reference to compute dot(Jacobian)*u from acceleration(dot(u)=0)
    ptr<MultiBodyTree> tree1(CreateMultiBodyTree(creator));
    ASSERT_TRUE(0x0 != tree1);
    CloneTreeCreator clone(tree1.get());
    // tree2 is used to compute dot(Jacobian)*u using the calculateJacobian function
    ptr<MultiBodyTree> tree2(CreateMultiBodyTree(clone));
    ASSERT_TRUE(0x0 != tree2);

    const int ndofs = tree1->numDoFs();
    const int nbodies = tree1->numBodies();
    if (ndofs <= 0) {
        *max_error = 0;
        return;
    }

    vecx q(ndofs);
    vecx u(ndofs);
    vecx dot_u(ndofs);
    vecx zero(ndofs);
    setZero(zero);

    double max_lin_error = 0;
    double max_ang_error = 0;

    for (int loop = 0; loop < nloops; loop++) {
        for (int i = 0; i < q.size(); i++) {
            q(i) = b3RandRange(-B3_PI, B3_PI);
            u(i) = b3RandRange(-B3_PI, B3_PI);
        }

        EXPECT_EQ(0, tree1->calculateKinematics(q, u, zero));
        EXPECT_EQ(0, tree2->calculatePositionAndVelocityKinematics(q, u));
        EXPECT_EQ(0, tree2->calculateJacobians(q, u));

        for (int idx = 0; idx < nbodies; idx++) {
            vec3 tmp1, tmp2;
            vec3 diff;
            EXPECT_EQ(0, tree1->getBodyLinearAcceleration(idx, &tmp1));
            EXPECT_EQ(0, tree2->getBodyDotJacobianTransU(idx, &tmp2));
            diff = tmp1 - tmp2;
            double lin_error = maxAbs(diff);

            if (lin_error > max_lin_error) {
                max_lin_error = lin_error;
            }

            EXPECT_EQ(0, tree1->getBodyAngularAcceleration(idx, &tmp1));
            EXPECT_EQ(0, tree2->getBodyDotJacobianRotU(idx, &tmp2));
            diff = tmp1 - tmp2;
            double ang_error = maxAbs(diff);
            if (ang_error > max_ang_error) {
                max_ang_error = ang_error;
            }
        }
    }
    *max_error = max_ang_error > max_lin_error ? max_ang_error : max_lin_error;
}

void calculateJacobianError(const MultiBodyTreeCreator& creator, const int nloops,
                            double* max_error) {
    // tree1 is used as reference to compute the Jacobian from velocities with unit u vectors.
    ptr<MultiBodyTree> tree1(CreateMultiBodyTree(creator));
    ASSERT_TRUE(0x0 != tree1);
    // tree2 is used to compute the Jacobians using the calculateJacobian function
    CloneTreeCreator clone(tree1.get());
    ptr<MultiBodyTree> tree2(CreateMultiBodyTree(clone));
    ASSERT_TRUE(0x0 != tree2);

    const int ndofs = tree1->numDoFs();
    const int nbodies = tree1->numBodies();

    if (ndofs <= 0) {
        *max_error = 0;
        return;
    }

    vecx q(ndofs);
    vecx zero(ndofs);
    setZero(zero);
    vecx one(ndofs);

    double max_lin_error = 0;
    double max_ang_error = 0;

    for (int loop = 0; loop < nloops; loop++) {
        for (int i = 0; i < q.size(); i++) {
            q(i) = b3RandRange(-B3_PI, B3_PI);
        }

        EXPECT_EQ(0, tree2->calculatePositionKinematics(q));
        EXPECT_EQ(0, tree2->calculateJacobians(q));

        for (int idx = 0; idx < nbodies; idx++) {
            mat3x ref_jac_r(3, ndofs);
            mat3x ref_jac_t(3, ndofs);
            ref_jac_r.setZero();
            ref_jac_t.setZero();
            // this re-computes all jacobians for every body ...
            // but avoids std::vector<eigen matrix> issues
            for (int col = 0; col < ndofs; col++) {
                setZero(one);
                one(col) = 1.0;
                EXPECT_EQ(0, tree1->calculatePositionAndVelocityKinematics(q, one));
                vec3 vel, omg;
                EXPECT_EQ(0, tree1->getBodyLinearVelocity(idx, &vel));
                EXPECT_EQ(0, tree1->getBodyAngularVelocity(idx, &omg));
                setMat3xElem(0, col, omg(0), &ref_jac_r);
                setMat3xElem(1, col, omg(1), &ref_jac_r);
                setMat3xElem(2, col, omg(2), &ref_jac_r);
                setMat3xElem(0, col, vel(0), &ref_jac_t);
                setMat3xElem(1, col, vel(1), &ref_jac_t);
                setMat3xElem(2, col, vel(2), &ref_jac_t);
            }

            mat3x jac_r(3, ndofs);
            mat3x jac_t(3, ndofs);
            mat3x diff(3, ndofs);

            EXPECT_EQ(0, tree2->getBodyJacobianTrans(idx, &jac_t));
            EXPECT_EQ(0, tree2->getBodyJacobianRot(idx, &jac_r));
            sub(ref_jac_t,jac_t,&diff);
            double lin_error = maxAbsMat3x(diff);
            if (lin_error > max_lin_error) {
                max_lin_error = lin_error;
            }
            sub(ref_jac_r, jac_r,&diff);
            double ang_error = maxAbsMat3x(diff);
            if (ang_error > max_ang_error) {
                max_ang_error = ang_error;
            }
        }
    }
    *max_error = max_ang_error > max_lin_error ? max_ang_error : max_lin_error;
}

void calculateVelocityJacobianError(const MultiBodyTreeCreator& creator, const int nloops,
                                    double* max_error) {
    // tree1 is used as reference to compute the velocities directly
    ptr<MultiBodyTree> tree1(CreateMultiBodyTree(creator));
    ASSERT_TRUE(0x0 != tree1);
    // tree2 is used to compute the velocities via jacobians
    CloneTreeCreator clone(tree1.get());
    ptr<MultiBodyTree> tree2(CreateMultiBodyTree(clone));
    ASSERT_TRUE(0x0 != tree2);

    const int ndofs = tree1->numDoFs();
    const int nbodies = tree1->numBodies();

    if (ndofs <= 0) {
        *max_error = 0;
        return;
    }

    vecx q(ndofs);
    vecx u(ndofs);

    double max_lin_error = 0;
    double max_ang_error = 0;

    for (int loop = 0; loop < nloops; loop++) {
        for (int i = 0; i < q.size(); i++) {
            q(i) = b3RandRange(-B3_PI, B3_PI);
            u(i) = b3RandRange(-B3_PI, B3_PI);
        }

        EXPECT_EQ(0, tree1->calculatePositionAndVelocityKinematics(q, u));
        EXPECT_EQ(0, tree2->calculatePositionKinematics(q));
        EXPECT_EQ(0, tree2->calculateJacobians(q));

        for (int idx = 0; idx < nbodies; idx++) {
            vec3 vel1;
            vec3 omg1;
            vec3 vel2;
            vec3 omg2;
            mat3x jac_r2(3, ndofs);
            mat3x jac_t2(3, ndofs);

            EXPECT_EQ(0, tree1->getBodyLinearVelocity(idx, &vel1));
            EXPECT_EQ(0, tree1->getBodyAngularVelocity(idx, &omg1));
            EXPECT_EQ(0, tree2->getBodyJacobianTrans(idx, &jac_t2));
            EXPECT_EQ(0, tree2->getBodyJacobianRot(idx, &jac_r2));
            omg2 = jac_r2 * u;
            vel2 = jac_t2 * u;

            double lin_error = maxAbs(vel1 - vel2);
            if (lin_error > max_lin_error) {
                max_lin_error = lin_error;
            }
            double ang_error = maxAbs(omg1 - omg2);
            if (ang_error > max_ang_error) {
                max_ang_error = ang_error;
            }
        }
    }
    *max_error = max_ang_error > max_lin_error ? max_ang_error : max_lin_error;
}

// test nonlinear terms: dot(Jacobian)*u (linear and angular acceleration for dot_u ==0)
// from Jacobian calculation method and pseudo-numerically using via the kinematics method.
TEST(InvDynJacobians, JacDotJacU) {
    const int kNumLevels = 5;
#ifdef B3_USE_DOUBLE_PRECISION
	const double kMaxError = 1e-12;
#else
    const double kMaxError = 5e-5;
#endif
    const int kNumLoops = 20;
    for (int level = 0; level < kNumLevels; level++) {
        const int nbodies = BT_ID_POW(2, level);
        CoilCreator coil(nbodies);
        double error;
        calculateDotJacUError(coil, kNumLoops, &error);
        EXPECT_GT(kMaxError, error);
        DillCreator dill(level);
        calculateDotJacUError(dill, kNumLoops, &error);
        EXPECT_GT(kMaxError, error);
    }

    const int kRandomLoops = 100;
    const int kMaxRandomBodies = 128;
    for (int loop = 0; loop < kRandomLoops; loop++) {
        RandomTreeCreator random(kMaxRandomBodies);
        double error;
        calculateDotJacUError(random, kNumLoops, &error);
        EXPECT_GT(kMaxError, error);
    }
}

// Jacobians: linear and angular acceleration for dot_u ==0
// from Jacobian calculation method and pseudo-numerically using via the kinematics method.
TEST(InvDynJacobians, Jacobians) {
    const int kNumLevels = 5;
#ifdef B3_USE_DOUBLE_PRECISION
	const double kMaxError = 1e-12;
#else
	const double kMaxError = 5e-5;
#endif
	const int kNumLoops = 20;
    for (int level = 0; level < kNumLevels; level++) {
        const int nbodies = BT_ID_POW(2, level);
        CoilCreator coil(nbodies);
        double error;
        calculateJacobianError(coil, kNumLoops, &error);
        EXPECT_GT(kMaxError, error);
        DillCreator dill(level);
        calculateDotJacUError(dill, kNumLoops, &error);
        EXPECT_GT(kMaxError, error);
    }
    const int kRandomLoops = 20;
    const int kMaxRandomBodies = 16;
    for (int loop = 0; loop < kRandomLoops; loop++) {
        RandomTreeCreator random(kMaxRandomBodies);
        double error;
        calculateJacobianError(random, kNumLoops, &error);
        EXPECT_GT(kMaxError, error);
    }
}

// test for jacobian*u == velocity
TEST(InvDynJacobians, VelocitiesFromJacobians) {
    const int kRandomLoops = 20;
    const int kMaxRandomBodies = 16;
    const int kNumLoops = 20;
#ifdef B3_USE_DOUBLE_PRECISION
	const double kMaxError = 1e-12;
#else
	const double kMaxError = 5e-5;
#endif
	for (int loop = 0; loop < kRandomLoops; loop++) {
        RandomTreeCreator random(kMaxRandomBodies);
        double error;
        calculateVelocityJacobianError(random, kNumLoops, &error);
        EXPECT_GT(kMaxError, error);
    }
}
#endif

int main(int argc, char** argv) {
	b3Srand(1234);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
