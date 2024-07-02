% M = 0.486;
% m = 0.211;
% b = 0.1;
% I = 0.006;
% g = 9.81;
% l = 0.609;
% q = (M+m)*(I+m*l^2)-(m*l)^2;
% s = tf('s');
% P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);
% 
% % PID Controller
% Kp = 100;
% Ki = 1;
% Kd = 20;
% C = pid(Kp,Ki,Kd);
% T = feedback(P_pend,C);
% 
% % Plot response
% t=0:0.01:10;
% impulse(T,t)
% axis([0, 2.5, -0.2, 0.2]);
% title({'Response of Pendulum Position to an Impulse Disturbance';'under PID Control: Kp = 100, Ki = 1, Kd = 20'});

% % Plot cart response
% % P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);
% % T2 = feedback(1,P_pend*C)*P_cart;
% % t = 0:0.01:5;
% % impulse(T2, t);
% % title({'Response of Cart Position to an Impulse Disturbance';'under PID Control: Kp = 100, Ki = 1, Kd = 20'});


M = .486;
m = 0.211;
b = 0.1;
I = 0.006;
g = 9.81;
l = 0.609;

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs)
sys_tf = tf(sys_ss)

Kp = 1;
Ki = 0.1;
Kd = 0.05;

s = tf('s');
PID = Kp + Ki/s + Kd*s;

sys_cl = feedback(PID*sys, 1);

t = 0:0.01:10
r = ones(size(t));
[y, t, x] = lsim(sys_cl, r, t);
e = r - y;
U = Kp * e + Ki * cumsum(e) + Kd * gradient(e, t);

figure;
subplot(2,1,1);
plot(t, y);
title('Output Response');
xlabel('Time (s)');
ylabel('Output');

subplot(2,1,2);
plot(t, U);
title('Control Signal (PID Output)');
xlabel('Time (s)');
ylabel('Control Signal');

% Optionally, plot other relevant signals (e.g., reference input, error)

