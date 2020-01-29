function [pn, T] = Isotropic_norm(p)

%Compute the centroid of the point set
centroid =mean(p)';
pc = p - centroid'; %centered points

% Scale points to have average distance from the origin sqrt(2)
d_center = sqrt(sum((pc(:,1)+pc(:,2)).^2)); % Compute the distance of all points to [0 0]'
scale = sqrt(2)/mean(d_center);
pn = scale.*pc;

%Prepare the transformation matrix to de-normalize F or H
T = eye(3);
T(1,1)=scale; T(2,2)=scale;
T(1,3) = -scale*centroid(1); T(2,3) = -scale*centroid(2);



