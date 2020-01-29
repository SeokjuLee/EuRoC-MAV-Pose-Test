function F_est_de = eight_points_algorithm(p1,p2)

% Normalize the points
[pn1, T1] = Isotropic_norm(p1);
[pn2, T2] = Isotropic_norm(p2);

% Solve the fundamental matrix with normalized points
F_est = Solve_fundamental_matrix(pn1,pn2);

% Force rank 2!
[U, S, V] = svd(F_est);
F_est = U(:, 1) * S(1,1) * transpose(V(:, 1)) + U(:, 2) ...
    * S(2,2) * transpose(V(:, 2));

%De-normmalize F
F_est_de = (T2')*F_est*T1;

