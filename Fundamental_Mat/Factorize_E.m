function M = Factorize_E(E)

%Extract rotation/Translation from E
W = [0 -1 0; 1 0 0; 0 0 1];
Z = [0 1 0; -1 0 0; 0 0 0];
[U, S, V]=svd(E);
R1_est = U*W*V'; R2_est = U*W'*V';
if (det(R1_est)<0) R1_est = -R1_est; end
if (det(R2_est)<0) R2_est = -R2_est; end
T_estS = U*Z*U'; T_est = [T_estS(3,2) T_estS(1,3) T_estS(2,1)]'

% compute the 4 possible configuration
M(:,:,1) =  [R1_est T_est];
M(:,:,2) =  [R2_est T_est];
M(:,:,3) =  [R1_est -T_est];
M(:,:,4) =  [R2_est -T_est];


