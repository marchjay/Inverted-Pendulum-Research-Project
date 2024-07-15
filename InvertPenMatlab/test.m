gyrAngle = 1;
posDist = 1;
controlSignal = 1;
% 
% % Position Transfer Function
% num = [1.996 5.318e-15 -29.86];
% den = [1 0.1996 -20.81 -2.986 0];
% 
% % Angle Transfer Function
% num1 = [3.044 -1.458e-16];
% den1 = [1 0.1996 -20.81 -2.986];
% 
% % Position
% G = tf(num, den);
% 
% % Angle
% C = tf(num1, den1);
% 
% % PID parameters for the Position Controller
% Kp1 = 100;
% Ki1 = 1;
% Kd1 = 20;
% 
% % PID parameters for the Angle Controller
% Kp2 = 100;
% Ki2 = 1;
% Kd2 = 1;
% 
% % Create Position Controller
% controllerP = pid(Kp1, Ki1, Kd1);
% 
% % Create Angle Controller
% controllerA = pid(Kp2, Ki2, Kd2);
% 
% T1 = feedback(controllerP, 0, posDist, control1);
% T2 = feedback(controllerA, 0, gyrAngle);
% 
% step(T1)

Kp = 100;
Ki = 1;
Kd = 1;

C = pid(Kp, Ki, Kd);
tf(C)

% Position Transfer Function
num = [1.996 5.318e-15 -29.86];
den = [1 0.1996 -20.81 -2.986 0];
 
% Position
G = tf(num, den)


T = feedback(C, G, posDist, controlSignal, -1);
size(controlSignal)









