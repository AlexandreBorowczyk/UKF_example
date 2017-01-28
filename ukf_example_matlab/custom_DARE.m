%function [ K ] = custom_DARE( A, B, Q, R )
%CUSTOM_DARE Summary of this function goes here
%   Homemade implementation of DARE
clear
clc

sys = ss([1,0;0,-2],[1;0],eye(2),[0;0]);

n = length(sys.A)*2;

I = eye(n/2);

O = zeros(n/2);

% Continous
if(0)
    disp('Continous')
    
    A = sys.A;
    B = sys.B;
    
    Q = I;
    R = 1;
    
    M = [ A,  -B*B';
        -Q,  -A'];
    
    L = [I,   O
         O, I];
    
    
    % Real Schur Decomposition
    [U,T] = schur(M)
    
    
    eig_values = diag(T);
    
    stable_eig_values = sort(eig_values(eig_values < 0));
    
    % Ordering Eigenvalues in the Real Schur Form
    while(sort(eig_values(1:n/2)) ~= stable_eig_values)
        for k = 1:n-1
            if all(eig_values(k) ~= stable_eig_values) && any(eig_values(k+1) == stable_eig_values)
                x = [T(k,k+1), T(k+1,k+1)-T(k,k)]';
                [G,y] = planerot(x);
                T(k:k+1,k:n) = G * T(k:k+1,k:n);
                T(1:k+1,k:k+1)= T(1:k+1,k:k+1) * G';
                U(1:n,k:k+1) = U(1:n,k:k+1) * G';
            end
            eig_values = diag(T);
        end
    end
    
    U
    T
    
    X = U(3:4,1:2)/U(1:2,1:2)
    
    K = -B'*X;
    [X,~,~] = care(A,B,Q);
    
    
    X
    
else
    % Discrete
    disp('Discrete')
    
    dsys = c2d(sys,0.01);
    
    A = dsys.A;
    B = dsys.B;
   
    Q = I;
    R = 1;
    
    
    
    M = [ A,  O;
         -Q,  I];
    
    L = [I,   B/R*B';
         O, A'];
    
    G = B/R*B';
     
    M = [A+G/A'*Q, -G/A';
         -A'\Q, inv(A')]
    
    % Real Schur Decomposition
    % L\M
    [U,T] = schur(M)
    
    eig_values = diag(T);
    
    stable_eig_values = sort(eig_values(eig_values < 1));
    
    
    % Ordering Eigenvalues in the Real Schur Form
    while(sort(eig_values(1:n/2)) ~= stable_eig_values)
        for k = 1:n-1
            if all(eig_values(k) ~= stable_eig_values) && any(eig_values(k+1) == stable_eig_values)
                x = [T(k,k+1), T(k+1,k+1)-T(k,k)]';
                [G,y] = planerot(x);
                T(k:k+1,k:n) = G * T(k:k+1,k:n);
                T(1:k+1,k:k+1)= T(1:k+1,k:k+1) * G';
                U(1:n,k:k+1) = U(1:n,k:k+1) * G';
            end
            eig_values = diag(T);
        end
    end
    
    U
    T
    
    X = U(3:4,1:2)/U(1:2,1:2)
    
    [X,~,~] = dare(A,B,Q);
    
    X
end


%end


% Regarding QZ
% := [S,BB,-U',U] = qz(M,L)
% [AA,BB,Qq,Zq] = qz(M,L)

% NOTE: Eigenvalue are not correctly ordered

