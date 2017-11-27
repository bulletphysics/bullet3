% License: Bullet3 license
% Author: Avik De <avikde@gmail.com>

robot = importrobot('../../data/TwoJointRobot_wo_fixedJoints.urdf');
show(robot)

robot.DataFormat = 'column';

X0 = zeros(4,1);

options = odeset('MaxStep',5e-3);
[t,X] = ode45(@(t, X) myDyn(t, X, robot), [0,5], X0, options);


subplot(211)
hold all
plot(t, X(:,1))
plot(b3output500(:,1), b3output500(:,2), '--')
plot(t, X(:,2))
plot(b3output500(:,1), b3output500(:,3), '--')
legend('m1','b1','m2','b2')
ylabel('pos')
subplot(212)
hold all
plot(t, X(:,3))
plot(b3output500(:,1), b3output500(:,4), '--')
plot(t, X(:,4))
plot(b3output500(:,1), b3output500(:,5), '--')
legend('m1','b1','m2','b2')
ylabel('vel')

hold off

function Xd = myDyn(t, X, robot)
	q = X(1:2);
	qd = X(3:4);
	qdd = forwardDynamics(robot, q, qd, [0;0.5*sin(10*t)]);
	Xd = [qd;qdd];
end
