close all
clear
clc

s = tf('s');
Ts = 10e-3;
z = tf('z',Ts);

Gcont = 0.48/(1 + 0.068*s + 0.0016*s^2);
G = zpk(minreal(c2d(Gcont,Ts,'zoh'), 1e-2));

[z,p,k] = zpkdata(G,'v');

l1 = 0;
l2 = 1;
l = 1;

% transient
s_hat = 0.2;
zeta = abs(log(s_hat))/(sqrt(pi^2 + log(s_hat)^2))*1.2; % con 1.2 soddisfa overshoot
ts = 1;
wn = 4.6/zeta;

% pzmap(G);
% zgrid(zeta,[]);

A = conv([1 -p(1)], [1 -p(2)]);
B = k*[1 -z];

A_plus = A;
A_minus = 1;
B_plus = 1;
B_minus = B;

degA = 2;
degAplus = 2;
degAminus = 0;


degS1 = l + degAminus
degR2 = degA
degAm = l + degA - degAminus

p1c = -zeta*wn + 1j*wn*sqrt(1 - zeta^2);
p1 = exp(p1c*Ts);

p2c = -zeta*wn - 1j*wn*sqrt(1 - zeta^2);
p2 = exp(p2c*Ts);

p3c = -30*wn*zeta; % per soddisfare settling time -> lo sparo lontanissimo dall'asse immaginario (in TC)
p3 = exp(p3c*Ts);


Am = poly([p1 p2 p3]);

Aplus_1 = polyval(A_plus,1);
G_1 = dcgain(G);
extra_eq = [0.125*ones(1,3) -Aplus_1*G_1*ones(1,2)];

Adioph = [1 -1]
Bdioph = B

M_S = [[Adioph(:);0;0] [0;Adioph(:);0] [0;0;Adioph(:)] [0;Bdioph(:);0] [0;0;Bdioph(:)] ; 
    extra_eq]
Gamma = [Am(:);0];
Theta = M_S\Gamma;
R2 = Theta(1:degR2+1);
S1 = Theta(degR2+2:end);

% build C
R = conv([1 -1], R2);
S = conv(A_plus, S1);
C = zpk(minreal(tf(S(:)',R(:)', Ts), 1e-2))


% simulation

t_sim = 5;
rho = 0.5;
delta_2 = 0;
out = sim("sxxxxxx_sim.slx");
r = out.r;
y = out.y;

figure
plot(r.time, r.data, 'r--');
hold on
plot(y.time, y.data, 'LineWidth', 1.5);

e_inf = y.data(end) - r.data(end)

clear out

rho = 0;
delta_2 = 1;
out = sim("sxxxxxx_sim.slx");
y = out.y;

figure
plot(y.time, y.data, 'LineWidth', 1.5);

