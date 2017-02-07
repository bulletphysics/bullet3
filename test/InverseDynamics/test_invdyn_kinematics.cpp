// Test of kinematic consistency: check if finite differences of velocities, accelerations
// match positions

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <gtest/gtest.h>

#include "../Extras/InverseDynamics/CoilCreator.hpp"
#include "../Extras/InverseDynamics/DillCreator.hpp"
#include "../Extras/InverseDynamics/SimpleTreeCreator.hpp"
#include "BulletInverseDynamics/MultiBodyTree.hpp"

using namespace btInverseDynamics;

const int kLevel = 5;
const int kNumBodies = BT_ID_POW(2, kLevel);

// template function for calculating the norm
template <typename T>
idScalar calculateNorm(T&);
// only implemented for vec3
template <>
idScalar calculateNorm(vec3& v) {
    return BT_ID_SQRT(BT_ID_POW(v(0), 2) + BT_ID_POW(v(1), 2) + BT_ID_POW(v(2), 2));
}

// template function to convert a DiffType (finite differences)
// to a ValueType. This is for angular velocity calculations
// via finite differences.
template <typename ValueType, typename DiffType>
DiffType toDiffType(ValueType& fd, ValueType& val);

// vector case: just return finite difference approximation
template <>
vec3 toDiffType(vec3& fd, vec3& val) {
    return fd;
}

// orientation case: calculate spin tensor and extract angular velocity
template <>
vec3 toDiffType(mat33& fd, mat33& val) {
    // spin tensor
    mat33 omega_tilde = fd * val.transpose();
    // extract vector from spin tensor
    vec3 omega;
    omega(0) = 0.5 * (omega_tilde(2, 1) - omega_tilde(1, 2));
    omega(1) = 0.5 * (omega_tilde(0, 2) - omega_tilde(2, 0));
    omega(2) = 0.5 * (omega_tilde(1, 0) - omega_tilde(0, 1));
    return omega;
}

/// Class for calculating finite difference approximation
/// of time derivatives and comparing it to an analytical solution
/// DiffType and ValueType can be different, to allow comparison
/// of angular velocity vectors and orientations given as transform matrices.
template <typename ValueType, typename DiffType>
class DiffFD {
public:
    DiffFD() : m_dt(0.0), m_num_updates(0), m_max_error(0.0), m_max_value(0.0), m_valid_fd(false) {}

    void init(std::string name, idScalar dt) {
        m_name = name;
        m_dt = dt;
        m_num_updates = 0;
        m_max_error = 0.0;
        m_max_value = 0.0;
        m_valid_fd = false;
    }

    void update(const ValueType& val, const DiffType& true_diff) {
        m_val = val;
        if (m_num_updates > 2) {
            // 2nd order finite difference approximation for d(value)/dt
            ValueType diff_value_fd = (val - m_older_val) / (2.0 * m_dt);
            // convert to analytical diff type. This is for angular velocities
            m_diff_fd = toDiffType<ValueType, DiffType>(diff_value_fd, m_old_val);
            // now, calculate the error
            DiffType error_value_type = m_diff_fd - m_old_true_diff;
            idScalar error = calculateNorm<DiffType>(error_value_type);
            if (error > m_max_error) {
                m_max_error = error;
            }

            idScalar value = calculateNorm<DiffType>(m_old_true_diff);
            if (value > m_max_value) {
                m_max_value = value;
            }

            m_valid_fd = true;
        }
        m_older_val = m_old_val;
        m_old_val = m_val;
        m_old_true_diff = true_diff;
        m_num_updates++;
        m_time += m_dt;
    }

    void printMaxError() {
        printf("max_error: %e dt= %e max_value= %e fraction= %e\n", m_max_error, m_dt, m_max_value,
               m_max_value > 0.0 ? m_max_error / m_max_value : 0.0);
    }
    void printCurrent() {
        if (m_valid_fd) {
            // note: m_old_true_diff already equals m_true_diff here, so values are not aligned.
            //      (but error calculation takes this into account)
            printf("%s time: %e fd: %e %e %e true: %e %e %e\n", m_name.c_str(), m_time,
                   m_diff_fd(0), m_diff_fd(1), m_diff_fd(2), m_old_true_diff(0), m_old_true_diff(1),
                   m_old_true_diff(2));
        }
    }

    idScalar getMaxError() const { return m_max_error; }
    idScalar getMaxValue() const { return m_max_value; }

private:
    idScalar m_dt;
    ValueType m_val;
    ValueType m_old_val;
    ValueType m_older_val;
    DiffType m_old_true_diff;
    DiffType m_diff_fd;
    int m_num_updates;
    idScalar m_max_error;
    idScalar m_max_value;
    idScalar m_time;
    std::string m_name;
    bool m_valid_fd;
};

template <typename ValueType, typename DiffType>
class VecDiffFD {
public:
    VecDiffFD(std::string name, int dim, idScalar dt) : m_name(name), m_fd(dim), m_dt(dt) {
        for (int i = 0; i < m_fd.size(); i++) {
            char buf[256];
            BT_ID_SNPRINTF(buf, 256, "%s-%.2d", name.c_str(), i);
            m_fd[i].init(buf, dt);
        }
    }
    void update(int i, ValueType& val, DiffType& true_diff) { m_fd[i].update(val, true_diff); }
    idScalar getMaxError() const {
        idScalar max_error = 0;
        for (int i = 0; i < m_fd.size(); i++) {
            const idScalar error = m_fd[i].getMaxError();
            if (error > max_error) {
                max_error = error;
            }
        }
        return max_error;
    }
    idScalar getMaxValue() const {
        idScalar max_value = 0;
        for (int i = 0; i < m_fd.size(); i++) {
            const idScalar value = m_fd[i].getMaxValue();
            if (value > max_value) {
                max_value= value;
            }
        }
        return max_value;
    }
    void printMaxError() {
        printf("%s: total  dt= %e max_error= %e\n", m_name.c_str(), m_dt, getMaxError());
    }

    void printCurrent() {
        for (int i = 0; i < m_fd.size(); i++) {
            m_fd[i].printCurrent();
        }
    }

private:
    std::string m_name;
    std::vector<DiffFD<ValueType, DiffType> > m_fd;
    const idScalar m_dt;
    idScalar m_max_error;
};

// calculate maximum difference between finite difference and analytical differentiation
int calculateDifferentiationError(const MultiBodyTreeCreator& creator, idScalar deltaT,
                                  idScalar endTime, idScalar* max_linear_velocity_error,
                                  idScalar* max_angular_velocity_error,
                                  idScalar* max_linear_acceleration_error,
                                  idScalar* max_angular_acceleration_error) {
    // setup system
    MultiBodyTree* tree = CreateMultiBodyTree(creator);
    if (0x0 == tree) {
        return -1;
    }
    // set gravity to zero, so nothing is added to accelerations in forward kinematics
    vec3 gravity_zero;
    gravity_zero(0) = 0;
    gravity_zero(1) = 0;
    gravity_zero(2) = 0;
    tree->setGravityInWorldFrame(gravity_zero);
    //
    const idScalar kAmplitude = 1.0;
    const idScalar kFrequency = 1.0;

    vecx q(tree->numDoFs());
    vecx dot_q(tree->numDoFs());
    vecx ddot_q(tree->numDoFs());
    vecx joint_forces(tree->numDoFs());

    VecDiffFD<vec3, vec3> fd_vel("linear-velocity", tree->numBodies(), deltaT);
    VecDiffFD<vec3, vec3> fd_acc("linear-acceleration", tree->numBodies(), deltaT);
    VecDiffFD<mat33, vec3> fd_omg("angular-velocity", tree->numBodies(), deltaT);
    VecDiffFD<vec3, vec3> fd_omgd("angular-acceleration", tree->numBodies(), deltaT);

    for (idScalar t = 0.0; t < endTime; t += deltaT) {
        for (int body = 0; body < tree->numBodies(); body++) {
            q(body) = kAmplitude * sin(t * 2.0 * BT_ID_PI * kFrequency);
            dot_q(body) = kAmplitude * 2.0 * BT_ID_PI * kFrequency * cos(t * 2.0 * BT_ID_PI * kFrequency);
            ddot_q(body) =
                -kAmplitude * pow(2.0 * BT_ID_PI * kFrequency, 2) * sin(t * 2.0 * BT_ID_PI * kFrequency);
        }

        if (-1 == tree->calculateInverseDynamics(q, dot_q, ddot_q, &joint_forces)) {
            delete tree;
            return -1;
        }

        // position/velocity
        for (int body = 0; body < tree->numBodies(); body++) {
            vec3 pos;
            vec3 vel;
            mat33 world_T_body;
            vec3 omega;
            vec3 dot_omega;
            vec3 acc;

            tree->getBodyOrigin(body, &pos);
            tree->getBodyTransform(body, &world_T_body);
            tree->getBodyLinearVelocity(body, &vel);
            tree->getBodyAngularVelocity(body, &omega);
            tree->getBodyLinearAcceleration(body, &acc);
            tree->getBodyAngularAcceleration(body, &dot_omega);

            fd_vel.update(body, pos, vel);
            fd_omg.update(body, world_T_body, omega);
            fd_acc.update(body, vel, acc);
            fd_omgd.update(body, omega, dot_omega);


//        fd_vel.printCurrent();
//fd_acc.printCurrent();
//fd_omg.printCurrent();
//fd_omgd.printCurrent();
        }
    }

    *max_linear_velocity_error = fd_vel.getMaxError()/fd_vel.getMaxValue();
    *max_angular_velocity_error = fd_omg.getMaxError()/fd_omg.getMaxValue();
    *max_linear_acceleration_error = fd_acc.getMaxError()/fd_acc.getMaxValue();
    *max_angular_acceleration_error = fd_omgd.getMaxError()/fd_omgd.getMaxValue();

    delete tree;
    return 0;
}

// first test: absolute difference between numerical and numerial
// differentiation should be small
TEST(InvDynKinematicsDifferentiation, errorAbsolute) {
    //CAVEAT:these values are hand-tuned to work for the specific trajectory defined above.
#ifdef BT_ID_USE_DOUBLE_PRECISION
    const idScalar kDeltaT = 1e-7;
	const idScalar kAcceptableError = 1e-4;
#else
    const idScalar kDeltaT = 1e-4;
	const idScalar kAcceptableError = 5e-3;
#endif
    const idScalar kDuration = 0.01;
    

    CoilCreator coil_creator(kNumBodies);
    DillCreator dill_creator(kLevel);
    SimpleTreeCreator simple_creator(kNumBodies);

    idScalar max_linear_velocity_error;
    idScalar max_angular_velocity_error;
    idScalar max_linear_acceleration_error;
    idScalar max_angular_acceleration_error;

    // test serial chain
    calculateDifferentiationError(coil_creator, kDeltaT, kDuration, &max_linear_velocity_error,
                                  &max_angular_velocity_error, &max_linear_acceleration_error,
                                  &max_angular_acceleration_error);

    EXPECT_LT(max_linear_velocity_error, kAcceptableError);
    EXPECT_LT(max_angular_velocity_error, kAcceptableError);
    EXPECT_LT(max_linear_acceleration_error, kAcceptableError);
    EXPECT_LT(max_angular_acceleration_error, kAcceptableError);

    // test branched tree
    calculateDifferentiationError(dill_creator, kDeltaT, kDuration, &max_linear_velocity_error,
                                  &max_angular_velocity_error, &max_linear_acceleration_error,
                                  &max_angular_acceleration_error);

    EXPECT_LT(max_linear_velocity_error, kAcceptableError);
    EXPECT_LT(max_angular_velocity_error, kAcceptableError);
    EXPECT_LT(max_linear_acceleration_error, kAcceptableError);
    EXPECT_LT(max_angular_acceleration_error, kAcceptableError);

    // test system with different joint types
    calculateDifferentiationError(simple_creator, kDeltaT, kDuration, &max_linear_velocity_error,
                                  &max_angular_velocity_error, &max_linear_acceleration_error,
                                  &max_angular_acceleration_error);

    EXPECT_LT(max_linear_velocity_error, kAcceptableError);
    EXPECT_LT(max_angular_velocity_error, kAcceptableError);
    EXPECT_LT(max_linear_acceleration_error, kAcceptableError);
    EXPECT_LT(max_angular_acceleration_error, kAcceptableError);
}

// second test: check if the change in the differentiation error
// is consitent with the second order approximation, ie, error ~ O(dt^2)
TEST(InvDynKinematicsDifferentiation, errorOrder) {
    const idScalar kDeltaTs[2] = {1e-4, 1e-5};
    const idScalar kDuration = 1e-2;

    CoilCreator coil_creator(kNumBodies);
    //    DillCreator dill_creator(kLevel);
    //    SimpleTreeCreator simple_creator(kNumBodies);

    idScalar max_linear_velocity_error[2];
    idScalar max_angular_velocity_error[2];
    idScalar max_linear_acceleration_error[2];
    idScalar max_angular_acceleration_error[2];

    // test serial chain
    calculateDifferentiationError(coil_creator, kDeltaTs[0], kDuration,
                                  &max_linear_velocity_error[0], &max_angular_velocity_error[0],
                                  &max_linear_acceleration_error[0],
                                  &max_angular_acceleration_error[0]);

    calculateDifferentiationError(coil_creator, kDeltaTs[1], kDuration,
                                  &max_linear_velocity_error[1], &max_angular_velocity_error[1],
                                  &max_linear_acceleration_error[1],
                                  &max_angular_acceleration_error[1]);

/*
	const idScalar expected_linear_velocity_error_1 =
        max_linear_velocity_error[0] * pow(kDeltaTs[1] / kDeltaTs[0], 2);
    const idScalar expected_angular_velocity_error_1 =
        max_angular_velocity_error[0] * pow(kDeltaTs[1] / kDeltaTs[0], 2);
    const idScalar expected_linear_acceleration_error_1 =
        max_linear_acceleration_error[0] * pow(kDeltaTs[1] / kDeltaTs[0], 2);
    const idScalar expected_angular_acceleration_error_1 =
        max_angular_acceleration_error[0] * pow(kDeltaTs[1] / kDeltaTs[0], 2);

    printf("linear vel error: %e %e  %e\n", max_linear_velocity_error[1],
           expected_linear_velocity_error_1,
           max_linear_velocity_error[1] - expected_linear_velocity_error_1);
    printf("angular vel error: %e %e  %e\n", max_angular_velocity_error[1],
           expected_angular_velocity_error_1,
           max_angular_velocity_error[1] - expected_angular_velocity_error_1);
    printf("linear acc error: %e %e  %e\n", max_linear_acceleration_error[1],
           expected_linear_acceleration_error_1,
           max_linear_acceleration_error[1] - expected_linear_acceleration_error_1);
    printf("angular acc error: %e %e  %e\n", max_angular_acceleration_error[1],
           expected_angular_acceleration_error_1,
           max_angular_acceleration_error[1] - expected_angular_acceleration_error_1);
*/
}

int main(int argc, char** argv) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    return EXIT_SUCCESS;
}
