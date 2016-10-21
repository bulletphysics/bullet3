#ifndef B3_ERPCFM_HELPER
#define B3_ERPCFM_HELPER

#include <LinearMath/btScalar.h>

/**
 * @brief		ERP/CFM Utils are to provide bullet specific helper functions.
 * @details		Details
 * @date		2015-09-20
 * @author		Benjamin Ellenberger
 */
class b3ERPCFMHelper {
public:

	/**
	 * == How To Use ERP and CFM ==
	 * ERP and CFM can be independently set in many joints. They can be set in contact joints, in joint limits and various other places, to control the spongyness and springyness of the joint (or joint limit).
	 * If CFM is set to zero, the constraint will be hard. If CFM is set to a positive value, it will be possible to violate the constraint by "pushing on it" (for example, for contact constraints by forcing the two contacting objects together). In other words the constraint will be soft, and the softness will increase as CFM increases. What is actually happening here is that the constraint is allowed to be violated by an amount proportional to CFM times the restoring force that is needed to enforce the constraint. Note that setting CFM to a negative value can have undesirable bad effects, such as instability. Don't do it.
	 * By adjusting the values of ERP and CFM, you can achieve various effects. For example you can simulate springy constraints, where the two bodies oscillate as though connected by springs. Or you can simulate more spongy constraints, without the oscillation. In fact, ERP and CFM can be selected to have the same effect as any desired spring and damper constants. If you have a spring constant k_p and damping constant k_d, then the corresponding ODE constants are:
	 *
	 * ERP = \frac{h k_p} {h k_p + k_d}
	 *
	 * CFM = \frac{1} {h k_p + k_d}
	 *
	 * where h is the step size. These values will give the same effect as a spring-and-damper system simulated with implicit first order integration.
	 * Increasing CFM, especially the global CFM, can reduce the numerical errors in the simulation. If the system is near-singular, then this can markedly increase stability. In fact, if the system is misbehaving, one of the first things to try is to increase the global CFM.
	 * @link http://ode-wiki.org/wiki/index.php?title=Manual:_All&printable=yes#How_To_Use_ERP_and_CFM
	 * @return
	 */
	/**
	 * Joint error and the Error Reduction Parameter (ERP)
	 *
	 * When a joint attaches two bodies, those bodies are required to have certain positions and orientations relative to each other. However, it is possible for the bodies to be in positions where the joint constraints are not met. This "joint error" can happen in two ways:
	 *
	 * If the user sets the position/orientation of one body without correctly setting the position/orientation of the other body.
	 * During the simulation, errors can creep in that result in the bodies drifting away from their required positions.
	 * Figure 3  shows an example of error in a ball and socket joint (where the ball and socket do not line up).
	 *
	 * There is a mechanism to reduce joint error: during each simulation step each joint applies a special force to bring its bodies back into correct alignment. This force is controlled by the error reduction parameter (ERP), which has a value between 0 and 1.
	 *
	 * The ERP specifies what proportion of the joint error will be fixed during the next simulation step. If ERP = 0 then no correcting force is applied and the bodies will eventually drift apart as the simulation proceeds. If ERP=1 then the simulation will attempt to fix all joint error during the next time step. However, setting ERP=1 is not recommended, as the joint error will not be completely fixed due to various internal approximations. A value of ERP=0.1 to 0.8 is recommended (0.2 is the default).
	 *
	 * A global ERP value can be set that affects most joints in the simulation. However some joints have local ERP values that control various aspects of the joint.
	 * @link http://ode-wiki.org/wiki/index.php?title=Manual:_All&printable=yes#How_To_Use_ERP_and_CFM
	 * @return
	 */
	static btScalar getERP(btScalar timeStep, btScalar kSpring,
		btScalar kDamper) {
		return timeStep * kSpring / (timeStep * kSpring + kDamper);
	}

	/**
	 * Most constraints are by nature "hard". This means that the constraints represent conditions that are never violated. For example, the ball must always be in the socket, and the two parts of the hinge must always be lined up. In practice constraints can be violated by unintentional introduction of errors into the system, but the error reduction parameter can be set to correct these errors.
	 * Not all constraints are hard. Some "soft" constraints are designed to be violated. For example, the contact constraint that prevents colliding objects from penetrating is hard by default, so it acts as though the colliding surfaces are made of steel. But it can be made into a soft constraint to simulate softer materials, thereby allowing some natural penetration of the two objects when they are forced together.
	 * There are two parameters that control the distinction between hard and soft constraints. The first is the error reduction parameter (ERP) that has already been introduced. The second is the constraint force mixing (CFM) value, that is described below.
	 *
	 * == Constraint Force Mixing (CFM) ==
	 * What follows is a somewhat technical description of the meaning of CFM. If you just want to know how it is used in practice then skip to the next section.
	 * Traditionally the constraint equation for every joint has the form
	 * \mathbf{J} v = \mathbf{c}
	 * where v is a velocity vector for the bodies involved, \mathbf{J} is a "Jacobian" matrix with one row for every degree of freedom the joint removes from the system, and \mathbf{c} is a right hand side vector. At the next time step, a vector \lambda is calculated (of the same size as \mathbf{c}) such that the forces applied to the bodies to preserve the joint constraint are:
	 * F_c = \mathbf{J}^T \lambda
	 * ODE adds a new twist. ODE's constraint equation has the form
	 *
	 * \mathbf{J} v = \mathbf{c} + \textbf{CFM} \, \lambda
	 * where CFM is a square diagonal matrix. CFM mixes the resulting constraint force in with the constraint that produces it. A nonzero (positive) value of CFM allows the original constraint equation to be violated by an amount proportional to CFM times the restoring force \lambda that is needed to enforce the constraint. Solving for \lambda gives
	 *
	 * (\mathbf{J} \mathbf{M}^{-1} \mathbf{J}^T + \frac{1}{h}  \textbf{CFM}) \lambda = \frac{1}{h} \mathbf{c}
	 * Thus CFM simply adds to the diagonal of the original system matrix. Using a positive value of CFM has the additional benefit of taking the system away from any singularity and thus improving the factorizer accuracy.
	 * @link http://ode-wiki.org/wiki/index.php?title=Manual:_All&printable=yes#How_To_Use_ERP_and_CFM
	 * @return
	 */
	static btScalar getCFM(btScalar avoidSingularity, btScalar timeStep, btScalar kSpring,
		btScalar kDamper) {
		return btScalar(avoidSingularity) / (timeStep * kSpring + kDamper);
	}
};

#endif /* B3_ERPCFM_HELPER */
