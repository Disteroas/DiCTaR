function [A_cal, B_cal, Q_cal, R_cal] = cal_m_builder(Hp, A, B, R, Q, S)

% funzione per creare le matrici in funzione di Hp

if (nargin == 5) % se non do S la creo uguale a Q
    S = Q;
end

n = size(A,1); % state dim
p = size(B,2); % input dim

% inizializzazione quantities
A_cal = A;
% per le altre matrici utilizzo cell arrays, con dimensione HpxHp, che
% alla fine trasformo in matrici. In questo modo si creano easy le matrici
% nei cicli for
B_cal = cell(Hp, Hp);
Q_cal = cell(Hp, Hp);
R_cal = cell(Hp, Hp);

for ii=1:Hp
    
    % A
    if ii>1
        A_cal = [A_cal; A^ii]; % semplice concatenazione 
    end
    
    
    for jj=1:Hp

        % B
        if (ii-jj >= 0) 
            B_cal{ii,jj} = A^(ii-jj)*B; % matrice triangolare inferiore diversa da zero
        else
            B_cal{ii,jj} = zeros(n,1); % mat tri sup piena di zeri
        end
        

        % Q e R
        if(ii == jj)

            % diagonale
            if(ii == Hp)
                Q_cal{ii,jj} = S;
                R_cal{ii,jj} = R;
            else
                Q_cal{ii,jj} = Q;
                R_cal{ii,jj} = R;
            end

        else
            % se non sono sulla diagonale metto gli zero
            Q_cal{ii,jj} = zeros(n,n);
            R_cal{ii,jj} = zeros(p,p);

        end
    end    
    
end

% converto in matrici 

B_cal = cell2mat(B_cal);
Q_cal = cell2mat(Q_cal);
R_cal = cell2mat(R_cal);

end