s = tf('s');

G = 1/((s + 2)*(s + 5));


%%
Kp = 100;
Kd = 10;
Ki=100;
PID = Kp + Ki/s + Kd*s;

T = feedback(PID*G,1);
hold on;
step(T)