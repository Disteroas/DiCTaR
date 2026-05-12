s_hat = 0.1;
zeta = abs(log(s_hat))/(sqrt(pi^2+log(s_hat)^2));

poles = [0.9+0.3j, 0.3+0.9j, 0.3+0.5j, 0.1+0.3j];

plot(poles, 'bo')
hold on
zgrid(zeta, 100)

%%
A = 0.732;
B = 0.219; 
x0 = 0.5; 
Hp = 3;
Q = 1;
R = 1;
n =size(A,1);
p = size(B,2);

Acal = [A;A^2;A^3];
Bcal = [[B zeros(n,1) zeros(n,1)]; [A*B B zeros(n,1)];[A^2*B A*B B]];
Qcal = diag([1,1,1]);
Rcal = diag([1,1,1]);

H = 2*(Bcal'*Qcal*Bcal + Rcal);
F = 2*Acal'*Qcal*Bcal;

U = -H\F'*x0


%% 
z = tf('z',1);
C = 0.2*(z+0.1)/(z-0.2);
G = 1/(z-1);
d1 = 2*z/(z-1);

W1 = zpk(minreal(G/(1+C*G),1e-2));

y = zpk(minreal(W1*d1, 1e-2));
[n, d] = tfdata(y, 'v');
[r, p, k] = residuez(n,d)


%% 

A = [0.05 0 -0.25; 0 -0.7 0; -0.25 0 0.5];
eig(A)
B = [1;0;3];

rank(ctrb(A,B))
Q = diag(ones(1,3));
Cq = chol(Q);
rank(obsv(A,Cq)) % check


%%
clear
A = [-0.6 0.8; -0.5 -0.2];
B = [1; 1];
x0 = [-10;-8];
Ts = 0.05;
usat = 3;

Hp = 4;
Q = [5 0; 0 1];
R = 1;
n = size(A,1);
p = size(B,2);

Acal = [A;A^2;A^3; A^4];
Bcal = [[B zeros(n,1) zeros(n,1), zeros(n,1)]; [A*B B zeros(n,1) zeros(n,1)];[A^2*B A*B B zeros(n,1)]; [A^3*B A^2*B A*B B]];
Qcal = blkdiag(Q,Q,Q,Q);
Rcal = blkdiag(R,R,R,R);
H = 2*(Bcal'*Qcal*Bcal + Rcal);
H = (H+H')/2;
F = 2*Acal'*Qcal*Bcal;

steps = 100;
G = [eye(Hp);-eye(Hp)];
h = usat*ones(2*Hp,1);
x_k = x0;
x_traj(:,1) = x0; % nota: deve essere per forza cosi, altrimenti quadprog si pianta, non posso invertire dimesioni di righe e colonne di x_traj

for kk=1:steps
    U = quadprog(H, x_k'*F, G, h);
    x_traj(:,kk+1) = A*x_k +B*U(1);
    x_k = x_traj(:,end);
    u_traj(kk) = U(1);

end


figure
stairs(0:Ts:steps*Ts, sqrt(x_traj(1,:).^2 + x_traj(2,:).^2))
yline(1e-4)