close all
clear
clc

addpath("MPCtools/");
A = [0 1; 0 -0.1];
B = [0; 0.1];
C = [1 0];
x0 = [0;0];
Ts = 50e-3;

sys = ss(A,B,C, 0);
sys_dt = c2d(sys,Ts,'zoh');
Ad = sys_dt.a;
Bd = sys_dt.b;
Cd = sys_dt.c;

Cy = eye(2);
Cz = Cd; Cc = Cz;
Dz = 0; Dc = 0;
Hp = 2.2/Ts + 5;
Hu = Hp;
Hw = 1;
zblk = 1;
ublk = 1;
du_max = [2];
du_min = [-2];
u_max = [10];
u_min = [-10];
z_max = [inf];
z_min = [-inf];
Q = 1;
R = 1;
W = [];
V = [];
h = Ts;
cmode = 0;
solver = 'qp_as';

md = MPCInit(Ad,Bd,Cy,Cz,Dz,Cc,Dc,Hp,Hw,zblk,Hu,ublk,du_max,du_min, ...
    u_max,u_min,z_max,z_min,Q,R,W,V,h,cmode,solver);

% simulation

sys_x = ss(A,B,eye(2),0);
t_sim = 5;

out = sim("sxxxxxx_sim_MPC.slx");

r = out.r;
y = out.y;

figure
plot(r.time, r.data, 'LineWidth',1.2);
hold on
plot(y.time, y.data,'LineWidth',1.2);

% tuning
Q = 100;
Hp = 2/Ts;
Hu = 10;
z_max = [1.02];
z_min = [-inf];

md = MPCInit(Ad,Bd,Cy,Cz,Dz,Cc,Dc,Hp,Hw,zblk,Hu,ublk,du_max,du_min, ...
    u_max,u_min,z_max,z_min,Q,R,W,V,h,cmode,solver);

% simulation

clear out;

out = sim("sxxxxxx_sim_MPC.slx");

r = out.r;
y = out.y;

figure
plot(r.time, r.data, 'LineWidth',1.2);
hold on
plot(y.time, y.data,'LineWidth',1.2);
xline(1.8)
xline(2.2)
