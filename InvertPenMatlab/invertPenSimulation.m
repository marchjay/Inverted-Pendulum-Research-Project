M = .486;
m = 0.211;
b = 0.1;
I = 0.006;
g = 9.81;
l = 0.609;

q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');

P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);

P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

sys_tf = [P_cart ; P_pend];

inputs = {'u'};
outputs = {'x'; 'phi'};

set(sys_tf,'InputName',inputs)
set(sys_tf,'OutputName',outputs)

sys_tf
% s = tf('s');
% PID = Kp + Ki/s + Kd*s;
% 
% sys_cl = feedback(PID*sys, 1);
% 
% t = 0:0.01:10
% r = ones(size(t));
% [y, t, x] = lsim(sys_cl, r, t);
% e = r - y;
% U = Kp * e + Ki * cumsum(e) + Kd * gradient(e, t);
% 
% figure;
% subplot(2,1,1);
% plot(t, y);
% title('Output Response');
% xlabel('Time (s)');
% ylabel('Output');
% 
% subplot(2,1,2);
% plot(t, U);
% title('Control Signal (PID Output)');
% xlabel('Time (s)');
% ylabel('Control Signal');
% 
% % Optionally, plot other relevant signals (e.g., reference input, error)

