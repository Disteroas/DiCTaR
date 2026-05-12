close all
clear
clc
addpath("MPCtools/");
A = [0 1; 0 -1];
B = [0; 1];
C = [1 0];
x0 = [1;0];

Ts = 0.05;
sys = ss(A,B,C,0);
Ce = eye(2);
sys_x = ss(A,B,Ce,0);
sysdt = c2d(sys,Ts,'zoh');
Ad = sysdt.a;
Bd = sysdt.b;

Q = diag([1 1]);
R = 1;
Hp = 2.5/Ts +5;
Hu = Hp;

md = MPCInit(Ad,Bd,Ce,Ce,[0;0],Ce,[0;0],Hp,1,1,Hu,1, ...
 		      3,-3,5.5,-5.5,[inf;inf], ...
 		      [-inf;-inf],Q,R,[],[],Ts,0,'qp_as');

Tend = 5;

out = sim("untitled.slx");

x = out.x;
u = out.u;

figure
plot(x.time, sqrt(x.data(:,1).^2+x.data(:,2).^2));
xline(2.5*(1.05))
xline(2.5*(0.95))
yline(1e-4)
figure
plot(u.time, u.data)

% tuning
Q = diag([2500 1]);

md = MPCInit(Ad,Bd,Ce,Ce,[0;0],Ce,[0;0],Hp,1,1,Hu,1, ...
 		      3,-3,5.5,-5.5,[inf;inf], ...
 		      [-inf;-inf],Q,R,[],[],Ts,0,'qp_as');

Tend = 5;
clear out
out = sim("untitled.slx");

x = out.x;
u = out.u;

figure
plot(x.time, sqrt(x.data(:,1).^2+x.data(:,2).^2));
xline(2.5*(1.05))
xline(2.5*(0.95))
yline(1e-4)
figure
plot(u.time, u.data)

%

Hp = 2.5/Ts;
Hu = 13;

md = MPCInit(Ad,Bd,Ce,Ce,[0;0],Ce,[0;0],Hp,1,1,Hu,1, ...
 		      3,-3,5.5,-5.5,[inf;inf], ...
 		      [-inf;-inf],Q,R,[],[],Ts,0,'qp_as');

Tend = 5;
clear out
out = sim("untitled.slx");

x = out.x;
u = out.u;

figure
plot(x.time, sqrt(x.data(:,1).^2+x.data(:,2).^2));
xline(2.5*(1.05))
xline(2.5*(0.95))
yline(1e-4)
figure
plot(u.time, u.data)
