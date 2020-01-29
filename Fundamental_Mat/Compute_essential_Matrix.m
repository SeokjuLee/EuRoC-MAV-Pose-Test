function E = Compute_essential_Matrix(p1,p2,K1,K2)

% convert to homogeneous coordinate system
p1h = [p1 ones(length(p1),1)]';
p2h = [p2 ones(length(p2),1)]';

% Project the points on the camera plane
Pc1 = inv(K1)*p1h; Pc1 = Pc1';
Pc2 = inv(K2)*p2h; Pc2 = Pc2';

%Isotropic normalization (not fully needed)
[p1n, T1] = Isotropic_norm(Pc1);
[p2n, T2] = Isotropic_norm(Pc2);

%Prepare the matrix A
A = [p2n(:,1).*p1n(:,1) p2n(:,1).*p1n(:,2) p2n(:,1)  p2n(:,2).*p1n(:,1) ...
    p2n(:,2).*p1n(:,2) p2n(:,2) p1n(:,1) p1n(:,2) ones(length(p1n),1)];

%Solve the essnetial matrix
[U, S, V]=svd(A);
sol = V(:,9);
E_est = reshape(sol,3,3)';

% Enforce the essential matrix rank constraints
[U S V]=svd(E_est);
s = (S(1,1) + S(2,2))/2;
S(1,1) = s; S(2,2)=s; S(3,3)=0;
E_est = U*S*V';

%De-normalize
E = T2'*E_est*T1;


