close all
clear
clc

Ts = 5e-3;
s = tf('s');
z = tf('z', Ts);
Gcont = 10*(s+25)/(s^2+25);
G = zpk(minreal(c2d(Gcont,Ts,'zoh'), 1e-2));
[zG,pG,kG] = zpkdata(G,'v')

s_hat = 0.2;
ts = 0.3;

zeta = abs(log(s_hat))/sqrt(pi^2+log(s_hat)^2)*1.8;
wn = 4.6/(zeta*ts);

% pzmap(G)
% zgrid(zeta, wn)
l1 = 0;
l2 = 1;
l = 1;

A = conv([1 -pG(1)], [1 -pG(2)]);
B = kG*[1 -zG(1)];
Aplus = 1;
Aminus = A;
Bplus = [1 -zG(1)];
Bminus = kG;

degS1 = 1 + 2 -1
degR2 = 2 - 1 -1
degAm = 1+2+2-1-1

p1c = -zeta*wn +1j*wn*sqrt(1-zeta^2)
p1 = exp(Ts*p1c);
p2c = -zeta*wn -1j*wn*sqrt(1-zeta^2)
p2 = exp(Ts*p2c);
p3c = -6*zeta*wn
p3 = exp(Ts*p3c);

Am = poly([p1 p2 p3])'
Adioph = conv([1 -1], Aminus)'
Bdioph = Bminus'

M_S = [Adioph [0;Bdioph;0;0] [0;0;Bdioph;0] [0;0;0;Bdioph]];
Theta = M_S\Am;
R2 = Theta(1:degR2+1);
S1 = Theta(degR2+2:end);

S = conv(Aplus,S1);
R = conv(Bplus, conv(R2, [1 -1]));

C = zpk(minreal(tf(S(:)', R(:)', Ts), 1e-2))

Tend = 5;
rho = 1;
delta1 = 0;

out = sim("dof_ex_1_sim.slx");
y = out.y;
r = out.r;
u = out.u;
max(u.data)
figure
plot(r.time,r.data, 'r--');
hold on
plot(y.time,y.data, 'b');

figure
plot(u.time, u.data)


clear out
rho = 0;
delta1 = 1;

out = sim("dof_ex_1_sim.slx");
y = out.y;
r = out.r;
u = out.u;
max(u.data)
figure
plot(y.time,y.data, 'b');