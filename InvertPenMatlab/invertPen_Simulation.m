% Parameters
m = 0.2;   % Mass of the pendulum (kg)
M = 0.5;   % Mass of the cart (kg)
b = 0.1;   % Damping coefficient of the cart (N/m/s)
I = 0.006; % Moment of inertia of the pendulum (kg*m^2)
g = 9.8;   % Gravitational acceleration (m/s^2)
l = 0.3;   % Length to the pendulum center of mass (m)

% Denominator for the A and B matrices
p = I*(M+m) + M*m*l^2;

% State-space matrices
A = [0 1 0 0;
     0 -(I + m*l^2)*b/p (m^2*g*l^2)/p 0;
     0 0 0 1;
     0 -(m*l*b)/p m*g*l*(M + m)/p 0];

B = [0;
     (I + m*l^2)/p;
     0;
     m*l/p];

C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

% LQR design
Q = diag([10, 1, 100, 1]); % State cost matrix
R = 0.01;                  % Input cost matrix
K = lqr(A, B, Q, R);

% Closed-loop system matrices
Acl = A - B*K;
Bcl = B;
Ccl = C;
Dcl = D;

% Simulation time
t = 0:0.01:10;

% Initial conditions (cart position, cart velocity, pendulum angle, pendulum angular velocity)
x0 = [0; 0; 0.1; 0];

% Closed-loop system
sys_cl = ss(Acl, Bcl, Ccl, Dcl);

% Simulate response to initial conditions
[y, t, x] = initial(sys_cl, x0, t);

% Plot results
figure;
subplot(3,1,1);
plot(t, x(:,1));
title('Cart Position');
xlabel('Time (s)');
ylabel('Position (m)');

subplot(3,1,2);
plot(t, x(:,3));
title('Pendulum Angle');
xlabel('Time (s)');
ylabel('Angle (rad)');

subplot(3,1,3);
plot(t, -K*x'); % Control input (force)
title('Control Force');
xlabel('Time (s)');
ylabel('Force (N)');
