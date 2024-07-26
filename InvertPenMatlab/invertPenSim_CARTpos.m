% Measured Parameters of Pendulum & Cart
M = .486;
m = 0.211;
b = 0.1;
I = 0.006;
g = 9.81;
l = 0.609;

% Plant definition for Pendulum
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');
P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

% Plant definition for Cart Position / PID Controller for Cart
P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);
% T2 = feedback(1,P_pend*C)*P_cart;

% Initialize and define PID controller for Pendulum
Kp = 100;
Ki = 1;
Kd = 20;
C = pid(Kp, Ki, Kd);
T = feedback(P_pend, C);


% Examine response of Pendulum with a step response / impulse
t = 0:0.01:10;
impulse(T2, t)
title({'Response of Pendulum Position to an Impulse Disturbance';'under PID Control: Kp = 1, Ki = 1, Kd = 1'});