%% ex 1

clear
clc

A = [0 1; 0.1 -0.3];
B = [0;1];
C = [7 7];
x0 = [0;0];

z = tf('z',1);
U = z/(z-1); %z transform of eps(k)

% OUTPUT
Y = zpk(minreal(C*inv(z*eye(size(A,1))-A)*(z*x0 + B*U),1e-2));
[nY, dY] = tfdata(Y(1), 'v');
[rY, pY, kY] = residuez(nY,dY) % -> then I anti transform

% STATE
X = zpk(minreal(inv(z*eye(size(A,1))-A)*(z*x0 + B*U),1e-2));
% first component
[nX1, dX1] = tfdata(X(1), 'v');
[rX1, pX1, kX1] = residuez(nX1,dX1) % -> then I anti transform
% second component
[nX2, dX2] = tfdata(X(2), 'v');
[rX2, pX2, kX2] = residuez(nX2,dX2) % -> then I anti transform

% MODAL ANALYSIS
lambda = eig(A) % -> 0.2^k and (-0.5)^k

% STABILITY
%internal -> see lambda -> internally stable
%bibo -> I have to compute H(z)
H = zpk(minreal(inv(z*eye(size(A,1))-A)*B,1e-2));
%[nH, dH] = tfdata(H(1), 'v');
%[rH, pH] = residuez(nH,dH)
[zH, pH, kH] = zpkdata(H(1), 'v') % -> i can use directly zpkdata to compute poles (NB I need to insert H(1), not just H)
% -> it's bibo stable


%% ex 2
clear
clc

A = [-0.2 0 0; 0 0 1; 0 0 0];
B = [1;1;1];
C = [1 0 0];
x0 = B;

z = tf('z',1);

X = zpk(minreal(z*inv(z*eye(size(A,1))-A)*x0, 1e-2))

% I obtain:
%
% X =
% 
%   From input to output...
%           z
%    1:  -------    -> x(k) = (-0.2)^k
%        (z+0.2)
% 
%        (z+1)
%    2:  -----     -> x(k) = delta(k) + delta(k-1)
%          z
% 
%    3:  1         -> x(k) = delta(k)


lambda = eig(A) % -> internally stable

H = zpk(minreal(inv(z*eye(size(A,1))-A)*B, 1e-2));
[zH, pH, kH] = zpkdata(H(1), 'v')