close all
clear
clc
addpath("MPCtools/")

A = [0 1; 900 0];
B = [0;-10];
C = [600 0];
x0 = [0; 0];
Ts = 0.001;
sys = ss(A,B,C,0);
sys_dt = c2d(sys,Ts,'zoh');

Ad = sys_dt.a;
Bd = sys_dt.b;
Cyd = eye(2);
Czd = sys_dt.c;
Dzd = 0;
Ccd = Czd;
Dcd = 0;
Hp = 40;
Hw = 1;
zblk = 1;
Hu = 40;
ublk = 1;
du_max = [inf];
du_min = [-inf];
u_max = [0.5];
u_min = [-0.5];
z_max = [inf];
z_min = [-inf];
Q = 1;
R = 1;
W = [];
V = [];
h = Ts;
cmode = 0;
solver = 'qp_as';
md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
 		      du_max,du_min,u_max,u_min,z_max, ...
 		      z_min,Q,R,W,V,h,cmode,solver);

% sim
Tend = 0.2;

sys_x = ss(A,B,eye(2), 0);

out = sim("mpc_ex_1.slx");
y = out.y;
r = out.r;

figure
plot(r.time,r.data,'r--');
hold on
plot(y.time, y.data,'b'); 

%% 
Q = 1000;
md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
 		      du_max,du_min,u_max,u_min,z_max, ...
 		      z_min,Q,R,W,V,h,cmode,solver);
clear out

out = sim("mpc_ex_1.slx");
y = out.y;
r = out.r;

figure
plot(r.time,r.data,'r--');
hold on
plot(y.time, y.data,'b'); 

%%
Q = 1;
z_max = [1.01];

md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
 		      du_max,du_min,u_max,u_min,z_max, ...
 		      z_min,Q,R,W,V,h,cmode,solver);
clear out

out = sim("mpc_ex_1.slx");
y = out.y;
r = out.r;

figure
plot(r.time,r.data,'r--');
hold on
plot(y.time, y.data,'b'); 
xline(0.035*(1.05))
xline(0.035*(0.95))

%%
Hp = 35; % min
md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
 		      du_max,du_min,u_max,u_min,z_max, ...
 		      z_min,Q,R,W,V,h,cmode,solver);
clear out

out = sim("mpc_ex_1.slx");
y = out.y;
r = out.r;

figure
plot(r.time,r.data,'r--');
hold on
plot(y.time, y.data,'b'); 
xline(0.035*(1.05))
xline(0.035*(0.95))