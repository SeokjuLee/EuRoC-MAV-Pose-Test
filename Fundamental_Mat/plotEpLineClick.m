function l = plotEpLineClick(F,im1,im2)

%Click points in the first image
figure;
NbPts = 8;
colors = lines(NbPts);
imshow(im1,[]); hold on;
[x,y] = ginput(NbPts);

%Plot points in the first image
pp = [x y ones(1,length(x))']
 for i=1:NbPts
     plot(pp(i,1),pp(i,2),'o','Color',colors(i,:),'LineWidth',5); 
 end


%Plot epipolar lines in the second image
 figure; imshow(im2,[]);
 for i=1:NbPts
    l = F*pp(i,:)';
    l1 = l(1); l2 = l(2); l3 = l(3);
    x=[1 size(im2,2)]; y = -(l1*x+l3)/l2; hold on;
    plot(x,y,'Color',colors(i,:),'LineWidth',5); 
 end