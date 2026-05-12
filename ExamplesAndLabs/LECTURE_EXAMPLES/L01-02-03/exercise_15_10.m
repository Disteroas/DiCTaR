% 15/10 

%% ex 1
clear
clc
z = tf('z', 1);
C = 0.007*(z+0.6)/(z-1);
G = (z+0.5)/((z-0.7)*(z-0.2));

% In general C and G are not provided in zpk form => I have to do it myself
C = zpk(C);
G = zpk(G);

% 1) check for unstable zero-pole cancellations (no unstable z-p canc
% found)
% 2) check for stability => I can study W(z) = L(z)/(1+L(z))
W_0  = zpk(C*G/(1+C*G));

% in this situation, which can be the degree of the denominator at most? It
% can be at max 1 (degree of C den) + 2 (degree of G den)
% from the computation, I get that W den has degree 6, but I can see that
% there are roots in common => I must use minreal!!

W = minreal(W_0)
% I can see that all poles of W have poles < 1 => the system is stable

% alternative method -> "block diagram semplification"

clear W;
L = series(C,G);
W = zpk(feedback(L,1,-1)) %the last -1 is for the sign
% in this case, feedback "contains" minreal inside itself, I don't need to
% put it after 


%% ex 2
clear
clc
% compute the tracking error response e(k) when
% --- r(k) = 0.1*eps(k) (eps = step response)
% --- d1(k) = 0,2*eps(k)
% --- d2(k) = 0

% firstly, we have to set the problem in the z domain, then we antitrasform

% the solution in z domain is made by superposition of r and d1 effect

% E(z) = W2(z)*R(z) - W1(z)*D1(z)

% computation by hand

z = tf('z',1);

C = zpk(0.007*(z+0.6)/(z-1)); % previous C and G
G = zpk((z+0.5)/((z-0.7)*(z-0.2)));
L = zpk(minreal(C*G, 1e-3)); 

R = minreal(0.1*z/(z-1), 1e-3);
D1 = minreal(0.2*z/(z-1), 1e-3);

W2 = zpk(minreal(1/(1+L), 1e-3));
W1 = zpk(minreal(G/(1+L), 1e-3));

E = zpk(minreal(W2*R - W1*D1, 1e-3))

[n_e, d_e] = tfdata(E,'v');
[r_e,p_e,k_e] = residuez(n_e, d_e)

% since k_e is a really low value => I can consider it = 0
% from the theory, I know that k = 0 if it is strictly proper (???)


%% ex 3
clear
clc
A = [-0.2 0 0; 0 0 1; 0 0 0];
B = [1;1;1];
C = [1 0 0];
x0 = [1;1;1];

z = tf('z', 1);

X = zpk(minreal(z*inv(z*eye(length(A))-A)*x0, 1e-3))

% then I can antitransform -> computation by hand

zI_A_1 = zpk(minreal(z*inv(z*eye(length(A))-A), 1e-3))
