function ImageD = RemoveDisto(Image, k, K, K_new)

nx = 1:size(Image,2); ny = 1:size(Image,1);
[xx, yy] = meshgrid(nx, ny);  

% 1. Projection on the camera plane
ptsD = inv(K)*[xx(:) yy(:) ones(length(xx(:)),1)]';

% 2. Apply distortion
r = ptsD(1,:).^2 + ptsD(2,:).^2;
xd(1,:) = ptsD(1,:).*(1 + k(1).*r + k(2).*r.^2);
xd(2,:) = ptsD(2,:).*(1 + k(1).*r + k(2).*r.^2);

% 3. Projection on the image plane
xd = K_new*[xd; ones(size(xd(1,:),2),1)'];

% 4. interpolation
out = interp2(Image, xd(1,:), xd(2,:), 'linear');
out(isnan(out))=0;%replace Nan
ImageD=reshape(out,length(ny),length(nx));