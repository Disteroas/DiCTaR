%% slide 33 cap 01
clear
clc
A = [3 0; -3.5 -0.5];
B = [1;2];
C = [1 -1];

x0 = [1;-2];
z = tf('z', 1); % 1 è il sample time -> perché z è definito solo ai sampling time

u = 2*z/(z-1); % input

% firstly I want to compute the zero input response
% i have to do a couple of nested operations
%   1. minreal -> since computation is performed numerically, such
%   numerical procedure are not so intelligent => it may happens that
%   sometimes there are common factors at numerator and den (that have to be
%   cancelled) (minreal performs this job under a tolerance, that is needed
%   to the algorithm to understand when it can consider equal two factors)
%   2. zpk -> since we want to do a partial fraction expansion, we need to
%   identify which are the roots at the denominator that make the single
%   terms -> I have to factorize the denominator

%% risposta libera
X_i = minreal(zpk(z*inv(z*eye(2)-A)*x0), 1e-3) 


% X_i =
% 
%   From input to output...
%          z
%    1:  -----  --- FIRST COMPONENT OF THE STATE
%        (z-3)
% 
%        -2 z (z-1.25)
%    2:  ------------- --- SECOND COMPONENT OF THE STATE
%        (z-3) (z+0.5)

% now I have to perform the step that reconduct me to the PFE

% in tfdata 'v' stands for vector format
[num1, den1] = tfdata(X_i(1), 'v'); % 1st component of the state

% in continuous time we use residue, here we use residuez because we are
% working in the z domain => the trick of divide/multiply by z is already computed by
% Matlab
% it is possible that function can be not stricly proper (same degree
% num/den), but PFE works only on stricly proper transfer functions

% k in residues stands for the case when I have to perform an inverse
% transormation of a proper fuction but not strictly -> k represents the
% quotient of polinomial division between num and den (if tf is not stricly
% proper I have number + ....)
[r1, p1, k1] = residuez(num1, den1)


[num2, den2] = tfdata(X_i(2), 'v'); % 2nd component of the state
[r2, p2, k2] = residuez(num2, den2)

%% risposta forzata

X_f = minreal(zpk(inv(z*eye(2)-A)*B*u), 1e-3) % 1st component

[num_f, den_f] = tfdata(X_f(1), 'v'); 
[rf, pf, kf] = residuez(num_f, den_f)

% same for second component



