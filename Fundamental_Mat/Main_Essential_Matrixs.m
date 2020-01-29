clear all; close all; clc;

% open images
% DataSeq = 'fountain-P11';
% DataSeq = 'castle-P30';
% DataSeq = 'entry-P10';
DataSeq = 'Herz-Jesus-P25';
Im1N = '0000.jpg';
Im2N = '0001.jpg';
im1 = imread(['Data/' DataSeq '/images/' Im1N]);
im2 = imread(['Data/' DataSeq '/images/' Im2N]);


%Open extrinsic/Intrinsic
Cal1 = dlmread(['Data/' DataSeq '/gt_dense_cameras/' Im1N '.camera']);
Cal2 = dlmread(['Data/' DataSeq '/gt_dense_cameras/' Im2N '.camera']);
K1=Cal1(1:3,1:3); K2=Cal2(1:3,1:3); %intrinsic
R1 = Cal1(5:7,1:3); R2 = Cal2(5:7,1:3); %Rotation
T1 = Cal1(8,1:3)'; T2 = Cal2(8,1:3)'; %Translation
M1 = [R1 T1; 0 0 0 1]; M2 = [R2 T2; 0 0 0 1]; %Transform 4x4
M = inv(M1)*M2; M = inv(M); %Relative transformation
T = M(1:3,4); R = M(1:3,1:3);

%Fundamental matrix (GT)
F = inv(K2)'*skew(T)*R*inv(K1);
E = skew(T)*R;


%% eight-points algorithm
%convert to greyscale
im1g = rgb2gray(im1);
im2g = rgb2gray(im2);

% detect points
points1 = detectSURFFeatures(im1g,'MetricThreshold', 1000);
points2 = detectSURFFeatures(im2g, 'MetricThreshold', 1000);


% Match using photometric
[f1,vpts1] = extractFeatures(im1g,points1);
[f2,vpts2] = extractFeatures(im2g,points2);
%  indexPairs = matchFeatures(f1,f2, 'MatchThreshold', 90, 'MaxRatio', 0.85) ;
  indexPairs = matchFeatures(f1,f2) ;
matchedPoints1 = vpts1(indexPairs(:,1));
matchedPoints2 = vpts2(indexPairs(:,2));
figure; showMatchedFeatures(im1g,im2g,matchedPoints1,matchedPoints2);
legend('matched points 1','matched points 2');


% RANSAC
[F,inliersIndex] = estimateFundamentalMatrix(matchedPoints1,...
    matchedPoints2,'Method','RANSAC',...
    'NumTrials',100000,'DistanceThreshold',0.1)

InMatch1 = matchedPoints1(inliersIndex);
InMatch2 = matchedPoints2(inliersIndex);

figure; ax = axes;
showMatchedFeatures(im1g,im2g,InMatch1,InMatch2,'montage','Parent',ax);
title(ax, 'Candidate point matches');
legend(ax, 'Matched points 1','Matched points 2');

%Inliers points
p1 = InMatch1.Location;
p2 = InMatch2.Location;

% Plot points
figure; imshow(im1,[]); hold on;
plot(p1(:,1), p1(:,2),'or', 'MarkerSize', 5, 'LineWidth',5)

figure; imshow(im2,[]); hold on;
plot(p2(:,1), p2(:,2),'ob', 'MarkerSize', 5, 'LineWidth',5)

%Compute essential matrix
E_est = Compute_essential_Matrix(p1,p2,K1,K2)

%Extract projection extrinsics
M_est = Factorize_E(E_est)

%Add the intrinsic to form F and plot lines
F_est = inv(K2)'*E_est*inv(K1)
[U S V] = svd(F_est);
S(3,3)=0;
F_est = U*S*V';
idx = randperm(length(p1));
p1 = p1(idx(1:8),:);
p2 = p2(idx(1:8),:);
plotEpLines(F_est,im1,im2,[p1 ones(length(p1),1)],[p2 ones(length(p2),1)])

% F_est = eight_points_algorithm(p1,p2)
% 
% 
% 
% % plotEpLineClick(F_est,im1,im2)
% 
% %Pick random points to display
% %Select 7 points randomly
% idx = randperm(length(p1));
% p1 = p1(idx(1:7),:);
% p2 = p2(idx(1:7),:);
% 
% plotEpLines(F_est,im1,im2,[p1 ones(length(p1),1)],[p2 ones(length(p2),1)])
%  
% plotEpLines(F,im1,im2,[p1 ones(length(p1),1)],[p2 ones(length(p2),1)])