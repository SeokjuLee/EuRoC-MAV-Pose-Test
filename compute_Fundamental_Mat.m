function [F InMatch1 InMatch2] = compute_Fundamental_Mat(im1g,im2g,thresh)

% detect points
points1 = detectSURFFeatures(im1g,'MetricThreshold', thresh);
points2 = detectSURFFeatures(im2g, 'MetricThreshold', thresh);


% Match using photometric
[f1,vpts1] = extractFeatures(im1g,points1);
[f2,vpts2] = extractFeatures(im2g,points2);
%  indexPairs = matchFeatures(f1,f2, 'MatchThreshold', 90, 'MaxRatio', 0.85) ;
indexPairs = matchFeatures(f1,f2) ;
matchedPoints1 = vpts1(indexPairs(:,1));
matchedPoints2 = vpts2(indexPairs(:,2));


% RANSAC
[F,inliersIndex] = estimateFundamentalMatrix(matchedPoints1,...
    matchedPoints2,'Method','RANSAC',...
    'NumTrials',100000,'DistanceThreshold',0.1);

InMatch1 = matchedPoints1(inliersIndex);
InMatch2 = matchedPoints2(inliersIndex);

close; figure; ax = axes;
showMatchedFeatures(im1g,im2g,InMatch1,InMatch2,'montage','Parent',ax);
title(ax, 'Candidate point matches');
legend(ax, 'Matched points 1','Matched points 2');