%% RH just input constraints
close all
clear

A = [0.3 1.5; 0.5 -0.4];
B = [0;1];
x0 = [20;20];
Ts = 0.1
Acal = [A;A^2;A^3];
Bcal = [B zeros(2,1) zeros(2,1); A*B B zeros(2,1); A^2*B A*B B];

Q = [20 0; 0 5];
Qcal = blkdiag(Q,Q,Q);
Rcal = blkdiag(5,5,5);

H = 2*(Bcal'*Qcal*Bcal + Rcal);
H = (H+H')/2;
F = 2*Acal'*Qcal*Bcal;

G = [eye(3);-eye(3)];
h = 13*ones(6,1);

x_k = x0;

x_traj(:,1) = x0;

for kk = 1: 20
    U = quadprog(H, x_traj(:,kk)'*F,G,h);
    u_traj(kk) = U(1);
    x_traj(:,kk+1) = A*x_traj(:,kk)+B*U(1);
end

figure
stairs(0:Ts:(Ts*19), u_traj);

figure
stairs(0:Ts:Ts*20, sqrt(x_traj(1,:).^2 + x_traj(2,:).^2))


%% RH with state constraints
close all
clear

A = [0.9616 0.1878; -0.3756 0.8677];
B = [0.0192;0.1878];
x_k = [-0.3; 0.4];
Ts = 0.2;

Acal = [A;A^2;A^3];
Bcal = [B zeros(2,1) zeros(2,1); A*B B zeros(2,1); A^2*B A*B B];
Q = [5 0 ; 0 1];
Qcal = blkdiag(Q,Q,Q);
Rcal = blkdiag(0.1, 0.1, 0.1);

H = 2*(Bcal'*Qcal*Bcal + Rcal);
H = (H+H')/2;
F = 2*Acal'*Qcal*Bcal;

Gu = [eye(3);-eye(3)];
hu = 0.5*ones(6,1);
Gx = Bcal;
G = [Gu;Gx];

x_traj(:,1) = x_k;
xmax = [0;0.5];

for kk=1:40
    
    hx = -Acal*x_k + repmat(xmax,3,1);
    
    h = [hu;hx];
    U = quadprog(H,x_k'*F,G,h);
    x_traj(:,kk+1) = A*x_k + B*U(1);
    x_k = x_traj(:,end);
    
end

figure
stairs(0:Ts:40*Ts, sqrt(x_traj(1,:).^2+x_traj(2,:).^2))


%% RH with state constraints (again)
close all
clear
A = [0.9616 0.1878; -0.37656 0.8677];
B = [0.0192;0.1878];
x0 = [-0.3;0.4];
Ts = 0.2;
Q = [5 0; 0 1];
R = 0.1;
Acal = [A;A^2;A^3];
Bcal = [B zeros(2,1) zeros(2,1); A*B B zeros(2,1); A^2*B A*B B];
Qcal = blkdiag(Q,Q,Q);
Rcal = blkdiag(R,R,R);

H = 2*(Bcal'*Qcal*Bcal + Rcal);
H = (H+H')/2;
F = 2*Acal'*Qcal*Bcal;

Gu = [eye(3);-eye(3)];
Gx = Bcal;
G = [Gu;Gx];

hu = 0.5*ones(6,1);
x_traj(:,1) = x0;
xmax = [0;0.5];
for kk=1:40
    hx = -Acal*x_traj(:,kk) + repmat(xmax,3,1);
    h = [hu;hx];
    U = quadprog(H, x_traj(:,kk)'*F,G,h);
    x_traj(:,kk+1) = A*x_traj(:,kk) + B*U(1);

end

figure
stairs(0:Ts:40*Ts, sqrt(x_traj(1,:).^2+x_traj(2,:).^2));


%% 1dof responses and value at k-> inf
close all
clear
z = tf('z',1);

C = 0.01*(z-0.2)/(z-0.8);
G = (z-1)/(z^2-0.2*z+0.02);

L = minreal(zpk(C*G),1e-2);
W = minreal(zpk(L/(1+L)),1e-2);
R = 2*z/(z-1);
Yr = W*R;
[nY,dY] = tfdata(Yr,'v');
[r,p,k] = residuez(nY,dY)

% complex p

M = abs(r(3)); M2 = 2*M
phi = angle(r(3))
nu = abs(p(3))
theta = angle(p(3))


y_temp = minreal(zpk((z-1)*Yr),1e-2);
Y_inf = dcgain(y_temp)


W_1 = minreal(zpk(G/(1+L)),1e-2);
Yd1 = minreal(zpk(W_1*(z/(z-1))),1e-2);

y_temp = minreal(zpk((z-1)*Yd1),1e-2);

Yinf1 = dcgain(y_temp)


%% finite horizon unconstrained
close all
clear
A = [0.3 1.5; 0.5 -0.4];
B = [0;1];
x0 = [20;20];
Ts = 0.1;
Q = [20 0; 0 5];

Ac = [A;A^2;A^3];
Bc = [B zeros(2,1) zeros(2,1); A*B B zeros(2,1); A^2*B A*B B];
Qc = blkdiag(Q,Q,Q);
Rc = blkdiag(5,5,5);

H = 2*(Bc'*Qc*Bc + Rc);
F = 2*Ac'*Qc*Bc;

U = -H\(F'*x0)

%% finite horizon constrained 
close all
clear
A = [0.3 1.5; 0.5 -0.4];
B = [0; 1];
x0 = [20;20];
Ac = [A;A^2;A^3];
Bc = [B zeros(2,1) zeros(2,1); A*B B zeros(2,1); A^2*B A*B B];
Q = [20 0; 0 5];
Qc = blkdiag(Q,Q,Q);
Rc = blkdiag(5,5,5);

H = 2*(Bc'*Qc*Bc + Rc);
H = (H+H')/2;
F = 2*Ac'*Qc*Bc;

h = 13*ones(6,1);
G = [eye(3);-eye(3)];

U = quadprog(H, x0'*F,G,h)

X = Ac*x0 +Bc*U