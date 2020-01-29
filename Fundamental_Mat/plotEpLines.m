function l = plotEpLines(F,im1,im2,pp1,pp2)

NbPts = length(pp1);

%Click points in the first image
figure;
colors = lines(NbPts);
imshow(im1,[]); hold on;

%Plot points and lines in the first image
 for i=1:NbPts
    plot(pp1(i,1),pp1(i,2),'o','Color',colors(i,:),'LineWidth',8);  hold on;
    l = F'*pp2(i,:)';
    l1 = l(1); l2 = l(2); l3 = l(3);
    x=[1 size(im2,2)]; y = -(l1*x+l3)/l2; hold on;
    plot(x,y,'Color',colors(i,:),'LineWidth',5); hold on;
 end

pause(250/1000)
%Plot epipolar lines in the second image
 figure; imshow(im2,[]); hold on;
 for i=1:NbPts
    plot(pp2(i,1),pp2(i,2),'o','Color',colors(i,:),'LineWidth',8);  hold on;
    l = F*pp1(i,:)';
    l1 = l(1); l2 = l(2); l3 = l(3);
    x=[1 size(im2,2)]; y = -(l1*x+l3)/l2; hold on;
    plot(x,y,'Color',colors(i,:),'LineWidth',5); 
 end