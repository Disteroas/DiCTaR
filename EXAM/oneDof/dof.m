clear
clc
s = tf('s');
Ts = 2e-3;
z = tf('z',Ts);

Gcont = 20*(s+50)/(s^2+100);

G = c2d(Gcont, Ts, 'zoh');

G = minreal(zpk(G),1e-2)

[zg, pg, kg] = zpkdata(G,'v')

A = conv([1 -pg(1)], [1 -pg(2)]);
B = kg*[1 -zg];

l1 = 0;
l2 = 1;
l = 1;
G_1 = dcgain(G);

s_hat = 0.2;
ts = 0.5;

zeta = abs(log(s_hat))/sqrt(pi^2+log(s_hat)^2);
wn = 4.6/(zeta*ts);

% pzmap(G);
% zgrid(zeta,wn)

% cancello lo zero, non i poli

Aplus = 1;
Bplus = [1 -zg];
Bminus = kg;
Aminus = A;

degS1 = 1 + 2
degR2 = 2 - 1 
degAm = 1 + 2 + 2 - 1 

p1c = -zeta*wn + 1j*wn*sqrt(1-zeta^2);
p2c = -zeta*wn - 1j*wn*sqrt(1-zeta^2);
p3c = -5*zeta*wn;
p1 = exp(Ts*p1c);
p2 = exp(Ts*p2c);
p3 = exp(Ts*p3c);
p4 = p3;

Am = poly([p1 p2 p3 p4]);
Bplus_1 = polyval(Bplus,1);

extra_eq = [Bplus_1*Ts/0.1*ones(1,2) -ones(1,4)];

Ad = conv([1 -1], Aminus)'
Bd = Bminus'


Gamma = [Am(:);0]

M_S = [[[Ad;0] [0; Ad] [0;Bd;0;0;0] [0;0;Bd;0;0] [0;0;0;Bd;0] [0;0;0;0;Bd]];extra_eq]

Theta = M_S\Gamma

R2 = Theta(1:degR2+1);
S1 = Theta(degR2+2:end);

R = conv([1 -1], conv(Bplus,R2));
S = conv(Aplus, S1);

C = minreal(zpk(tf(S(:)',R(:)',Ts)),1e-2)

%% sim
Tend = 5;
rho = 1;
delta1 = 0;
delta2 = 0;

out = sim("dof_ex_1_sim.slx");

r = out.r;
y = out.y;

figure
plot(r.time,r.data,'r--');
hold on
plot(y.time,y.data,'b');


clear out
rho = 0;
delta1 = 1;
delta2 = 0;

out = sim("dof_ex_1_sim.slx");


y = out.y;

figure
plot(y.time,y.data,'b');



clear out
rho = 0;
delta1 = 0;
delta2 = 1;

out = sim("dof_ex_1_sim.slx");


y = out.y;

figure
plot(y.time,y.data,'b');
