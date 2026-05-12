close all
clear
clc

s = tf('s');
Ts = 5e-3;
z = tf('z',Ts);
Gcont = 40/(s^2+4*s-10);
G = c2d(Gcont,Ts,'zoh');

G = minreal(zpk(G),1e-2);
[zg, pg, kg] = zpkdata(G,'v')

A = conv([1 -pg(1)], [1 -pg(2)]);
B = kg*[1 -zg];

l1 = 0;
l2 = 1;
l = 1;

k = 0.1*Ts/5e-4;

s_hat = 0.25;
zeta = abs(log(s_hat))/(sqrt(pi^2+log(s_hat)^2));
wn1 = 4,6/zeta/2.5;
wn2 = (2.16*zeta+0.6)/0.9
wn = wn1;
% pzmap(G)
% zgrid(zeta,wn1)

Aminus = [1 -pg(1)];
Aplus = [1 -pg(2)];
Bminus = B;
Bplus = 1;

degS1 = 2;
degR2 = 2;
degAm = 4;

p1c = -wn*zeta + 1j*wn*sqrt(1-zeta^2);
p1 = exp(p1c*Ts);
p2c = -wn*zeta - 1j*wn*sqrt(1-zeta^2);
p2 = exp(p2c*Ts);
p3c = -5*wn*zeta;
p3 = exp(p3c*Ts);
p4 = p3;

Am = poly([p1 p2 p3 p4]);


Aminus_1 = polyval(Aminus,1);
extra_eq = [k*ones(1,3) -Aminus_1*dcgain(G)*ones(1,3)];

Ad = conv(Aminus, [1 -1])';
Bd = Bminus';

M_S = [[[Ad;0;0] [0;Ad;0] [0;0;Ad] [0;Bd;0;0] [0;0;Bd;0] [0;0;0;Bd]] ; extra_eq]

Theta = [Am(:);0];
Gamma = M_S\Theta



%% lab 4 es 1-2
close all
clear

A = [0 1; 0 -1];
B = [0;1];
C = [1 0];
Ts = 0.05;
sys = ss(A,B,C,0);
sysdt = c2d(sys,Ts,'zoh');
Ad = sysdt.a;
Bd = sysdt.b;
Cd = sysdt.c;

Aaug = [1 -Ts*Cd; zeros(2,1) Ad];
Baug = [0; Bd];
Q = diag([1 1 1]);
R = 1; 

rank(ctrb(Ad, Bd))

Cq = chol(Q);
rank(obsv(Aaug, Cq))

K = dlqr(Aaug, Baug, Q, R)

%% 
close all 
clear

A = [0 1; 0 -1];
B = [0;1];
C = [1 0];
sys = ss(A,B,C,0);
sysdt = c2d(sys,0.05,'zoh');
Ad = sysdt.a;
Bd = sysdt.b;
Cd = sysdt.c;


Aaug = [1 -0.05*Cd; zeros(2,1) Ad];
Baug = [0;Bd];

rank(ctrb(Ad,Bd))

Q = diag(ones(1, size(A,1)+1));
R = 1;

K = dlqr(Aaug,Baug,Q,R)

%%
(1-0.4)/(1-0.2);
%%
A = [0.9 0.7 0 0; -0.7 0.9 0 0; 0 0 -1 1; 0 0 0 1];
abs(eig(A))

%%
clear
z = tf('z',1);
G = zpk((z-0.1)/(z^2 -2.1*z+1.1));
C = 2*(z-1.1)/(z-0.5);
L = minreal(zpk(C*G),1e-2);
W = minreal(zpk(L/(1+L)),1e-2);
W1 = minreal(zpk(G/(1+L)),1e-2);
W2 = minreal(zpk(1/(1+L)),1e-2);
R = 1.2*z/(z-1);
D1 = 0.2*z/(z-1);
D2 = 0.25*z/(z-1);
Y = minreal(zpk(W*R+W1*D1+W2*D2),1e-2)
Ytemp = minreal(zpk((z-1)*Y),1e-2);
dcgain(Ytemp)

%% 
clear
z = tf('z',1);
C = 0.2*(z+0.1)/(z-0.2);
G = 1/(z-1);
L = minreal(zpk(C*G),1e-2);
W1 = minreal(zpk(G/(1+L)),1e-2);
D1 = 2*z/(z-1);
Y = minreal(zpk(W1*D1),1e-2)
[n, d] = tfdata(Y,'v');
[r, p, k] = residuez(n, d)

%%
clear
z = tf('z',1);
C = 2*(z-0.9)/(z-0.5);
G = zpk(2*(z-0.1)/(z^2-1.4*z+0.4))
L = minreal(zpk(C*G),1e-2);
W = minreal(zpk(L/(1+L)),1e-2)
W1 = minreal(zpk(G/(1+L)),1e-2);
W2 = minreal(zpk(1/(1+L)),1e-2);
R = 1.2*z/(z-1);
D1 = 0.2*z/(z-1);
D2 = 0.25*z/(z-1);
Y = minreal(zpk(W*R+W1*D1+W2*D2),1e-2)
Ytemp = minreal(zpk((z-1)*Y),1e-2);
dcgain(Ytemp)

%%
clear
z = tf('z',1);
C = (z-1.2)/(z-0.6);
G = (z-0.2)/((z-1)*z-1.2);
Ts = 1;
rho = 0;
d1 = 1;
d2 = 0;
G_ct = d2c(G);
Tend = 40;
out = sim("sim_with_disturbances.slx");

figure
plot(out.y.time, out.y.data)


%%
clear
close all
z = tf('z',1);
C = 0.4*(z-0.4)/(z-0.2);
G = (z+0.2)/((z-1)*(z-0.1));
L = minreal(zpk(C*G),1e-2);
W = minreal(zpk(L/(1+L)),1e-2)
Ts = 1;
rho = 0;
d1 = 0.15;
d2 = 0.1;
G_ct = d2c(G);
Tend = 40;
out = sim("sim_with_disturbances.slx");

figure
plot(out.y.time, out.y.data)
d1 = 0.15*z/(z-1);
W1 = minreal(zpk(G/(1+L)),1e-2);
Y = minreal(zpk(W1*d1),1e-2)
Ytemp = minreal(zpk((z-1)*Y),1e-2);
dcgain(Ytemp)


%% 
clear
close all
z = tf('z',1);
C = 1.5*(z- 0.7)/(z-0.9);
G = 0.22/(z-1.2);
W2 = zpk(minreal(1/(1+G*C),1e-2))

Y = zpk(minreal(W2*(z/(z-1)), 1e-2))

[n d] = tfdata(Y,'v');
[r p k] = residuez(n,d)

abs(r(2))
abs(p(2))
angle(r(2))
angle(p(2))

%%
clear
close all
z = tf('z',1);
G = 0.005*(z+0.96)/((z-1)*(z-0.9));
figure
pzmap(G)
zgrid(0.5, 100)


clear G
G = 15*(z+0.96)/((z-1)*(z-0.9));
figure
pzmap(G)
zgrid(0.5, 100)

%%
clear 
zeta = abs(log(0.12))/(sqrt(pi^2+log(0.12)^2));
wn = (2.16*zeta+0.6)/1.9
0.15/wn
0.33/wn

%% 
clear
zeta = abs(log(0.1))/(sqrt(pi^2+log(0.1)^2));
wn = 4.6/zeta
0.15/wn*1e3
0.33/wn*1e3

%%
clear
zeta = abs(log(0.1))/(sqrt(pi^2+log(0.1)^2));
poles = [0.9+0.3j 0.3+0.9j 0.3+0.5j 0.1+0.3j];
plot(poles,'bo');
zgrid(zeta, 100)