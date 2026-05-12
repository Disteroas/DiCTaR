close all
clear
clc

s = tf('s');
Ts = 0.01;
z = tf('z',Ts);
Gcont = 10/(1+s/50);
G = c2d(Gcont,Ts,'zoh')
[z,p,k] = zpkdata(G,'v')

% ss
l1 = 0;
l2 = 2;
G_1 = dcgain(G);
l = 2;

% transient
s_hat = 0.15;
ts = 0.8;

zeta = abs(log(s_hat))/(sqrt(pi^2 + log(s_hat)^2))*1.3;
wn = log(100/1)/(zeta*ts);
% pzmap(G)
% zgrid(zeta, wn);

A = [1 -p(1)];
B = k;
Aplus = A;
Aminus = 1;
Bplus = 1;
Bminus = B;

degS1 = l
degR2 = 1
degAm = l + 1

% poli

p1c = -wn*zeta + 1j*wn*sqrt(1-zeta^2); % prob DA RICORDARE
p1 = exp(Ts*p1c);
p2c = -wn*zeta - 1j*wn*sqrt(1-zeta^2);
p2 = exp(Ts*p2c);
p3c = -8*wn*zeta;
p3 = exp(Ts*p3c);

Am = poly([p1 p2 p3])

Ad = conv([1 -1], conv([1 -1], 1))' % gia incolon
Bd = B'
Aplus_1 = polyval(Aplus,1);
extra_eq = [2e-3*ones(1,2) -Aplus_1*ones(1,3)];
Gamma = [Am(:); 0];
M_S = [[Ad;0] [0;Ad] [0;Bd;0;0] [0;0;Bd;0] [0;0;0;Bd] ; extra_eq]

T = M_S\Gamma

R2 = T(1:degR2+1)
S1 = T(degR2+2:end)

R = conv([1 -1], conv([1 -1], R2))
S = conv(Aplus,S1)

C = zpk(minreal(tf(S(:)', R(:)', Ts), 1e-2))


% sim

rho = 1;
delta_1 = 0;
delta_2 = 0;
Tend = 5;

out = sim("lab3_sim.slx");
y = out.y;
r = out.r;
figure
plot(r.time, r.data, 'r--');
hold on
plot(y.time, y.data,'b')


clear out
rho = 0;
delta_1 = 1;
delta_2 = 0;
Tend = 5;

out = sim("lab3_sim.slx");
y = out.y;
figure
plot(y.time, y.data,'b')


clear out
rho = 0;
delta_1 = 0;
delta_2 = 1;
Tend = 5;

out = sim("lab3_sim.slx");
y = out.y;
figure
plot(y.time, y.data,'b')