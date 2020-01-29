%% Main ERUROC Test image and motion validity
clear all; close all; clc;
addpath('Fundamental_Mat')

%% Read the csv file containing the poses
% T = readtable('C:\Users\Seokju\Desktop\EuRoC-MAV\V1_01_easy\mav0\state_groundtruth_estimate0\data.csv');
% T = readtable('C:\Users\Seokju\Desktop\EuRoC-MAV\V1_01_easy\mav0\vicon0\data.csv');
T = readtable('C:\Users\Seokju\Desktop\EuRoC-MAV\MH_01_easy\mav0\state_groundtruth_estimate0\data.csv');
TablePose = T(:,2:8);
TableTsS = T(:,1);
PoseArray = table2array(TablePose);
TsSArray = table2array(TableTsS);

%% Transformation Body to camera!
% MBC = [ 0.0148655429818, -0.999880929698,   0.00414029679422, -0.0216401454975;
%         0.999557249008,   0.0149672133247,  0.025715529948,   -0.064676986768;
%        -0.0257744366974,  0.00375618835797, 0.999660727178,    0.00981073058949;
%         0.0, 0.0, 0.0, 1.0]
MBC = [ 0.0125552670891, -0.999755099723,  0.0182237714554, -0.0198435579556;
        0.999598781151,   0.0130119051815, 0.0251588363115,  0.0453689425024;
       -0.0253898008918,  0.0179005838253, 0.999517347078,   0.00786212447038;
        0.0, 0.0, 0.0, 1.0]

% MSB = [1.0, 0.0, 0.0,  7.48903e-02;
%        0.0, 1.0, 0.0, -1.84772e-02;
%        0.0, 0.0, 1.0, -1.20209e-01;
%        0.0, 0.0, 0.0,  1.0];
% MSB = [0.0,  0.0, 1.0,  7.48903e-02;
%        0.0, -1.0, 0.0, -1.84772e-02;
%        1.0,  0.0, 0.0, -1.20209e-01;
%        0.0,  0.0, 0.0,  1.0];
MSB = [ 0.33638, -0.01749,  0.94156,  0.06901;
       -0.02078, -0.99972, -0.01114, -0.02781;
        0.94150, -0.01582, -0.33665, -0.12395;
            0.0,      0.0,      0.0,      1.0];
     
%% Convert quaternion to rotation matrix
MS = {}; MB = {}; MC = {}; Poss_T = []; Body_T = []; Cam_T = [];

tic;
for i =1:length(PoseArray) 
%     RTemp = quat2mat(PoseArray(i,4:end));
%     T = PoseArray(i,1:3)';
%     MS{i} = [RTemp T; 0 0 0 1];
%     Poss_T = [Poss_T; MS{i}(1:3,4)'];
% 
%     %Transform into body ref
%     MB{i} = (MSB)*(MS{i});
%     Body_T = [Body_T; MB{i}(1:3,4)'];
    
    RTemp = quat2mat(PoseArray(i,4:end));
    T = PoseArray(i,1:3)';
    MB{i} = [RTemp T; 0 0 0 1];
    Body_T = [Body_T; MB{i}(1:3,4)'];
    
    %Tranform into cam
%     MC{i} = inv(MBC)*(MB{i});
%     MC{i} = (MBC)*(MB{i});
%     MC{i} = inv((MB{i})*(MBC));
%     MC{i} = inv(inv(MB{i})*(MBC));
%     MC{i} = inv((MB{i})*inv(MBC));
%     MC{i} = inv(inv(MB{i})*inv(MBC));
%     MC{i} = inv((MBC)*(MB{i}));
%     MC{i} = inv(inv(MBC)*(MB{i}));
%     MC{i} = inv((MBC)*inv(MB{i}));    % OK
    MC{i} = inv(inv(MBC)*inv(MB{i}));   % OK
    Cam_T = [Cam_T; MC{i}(1:3,4)']; 
end
elapsed = toc;
fprintf('<1> elapsed time: %.2f sec\n',elapsed);

%% Plot the position sensor position
% close all; 
% figure(1);
% endT = 5000;
% plot3(Poss_T(1:10:endT,1),Poss_T(1:10:endT,2),Poss_T(1:10:endT,3),'r.'); 
% axis equal; hold on;
% xlabel('x'); ylabel('y'); zlabel('z'); 
% plot3(Poss_T(1,1),Poss_T(1,2),Poss_T(1,3),'b.','MarkerSize',12);
% plot3(Poss_T(1*endT/4,1),Poss_T(1*endT/4,2),Poss_T(1*endT/4,3),'m.','MarkerSize',12);
% plot3(Poss_T(2*endT/4,1),Poss_T(2*endT/4,2),Poss_T(2*endT/4,3),'m.','MarkerSize',12);
% plot3(Poss_T(3*endT/4,1),Poss_T(3*endT/4,2),Poss_T(3*endT/4,3),'m.','MarkerSize',12);
% plot3(Poss_T(endT,1),Poss_T(endT,2),Poss_T(endT,3),'g.','MarkerSize',12);
% title('position sensor');

%% Plot the body position
% figure(2);
% endT = 5000;
% plot3(Body_T(1:10:endT,1),Body_T(1:10:endT,2),Body_T(1:10:endT,3),'r.'); 
% axis equal; hold on;
% xlabel('x'); ylabel('y'); zlabel('z'); 
% plot3(Body_T(1,1),Body_T(1,2),Body_T(1,3),'b.','MarkerSize',12);
% plot3(Body_T(1*endT/4,1),Body_T(1*endT/4,2),Body_T(1*endT/4,3),'m.','MarkerSize',12);
% plot3(Body_T(2*endT/4,1),Body_T(2*endT/4,2),Body_T(2*endT/4,3),'m.','MarkerSize',12);
% plot3(Body_T(3*endT/4,1),Body_T(3*endT/4,2),Body_T(3*endT/4,3),'m.','MarkerSize',12);
% plot3(Body_T(endT,1),Body_T(endT,2),Body_T(endT,3),'g.','MarkerSize',12);
% title('body');

%% Plot the cam0 position
% figure(3);
% endT = 5000;
% plot3(Cam_T(1:10:endT,1),Cam_T(1:10:endT,2),Cam_T(1:10:endT,3),'r.'); 
% axis equal; hold on;
% xlabel('x'); ylabel('y'); zlabel('z'); 
% plot3(Cam_T(1,1),Cam_T(1,2),Cam_T(1,3),'b.','MarkerSize',12);
% plot3(Cam_T(1*endT/4,1),Cam_T(1*endT/4,2),Cam_T(1*endT/4,3),'m.','MarkerSize',12);
% plot3(Cam_T(2*endT/4,1),Cam_T(2*endT/4,2),Cam_T(2*endT/4,3),'m.','MarkerSize',12);
% plot3(Cam_T(3*endT/4,1),Cam_T(3*endT/4,2),Cam_T(3*endT/4,3),'m.','MarkerSize',12);
% plot3(Cam_T(endT,1),Cam_T(endT,2),Cam_T(endT,3),'g.','MarkerSize',12);
% title('cam0');

%% Plot the body + cam0 position
figure(4);
endT = 5000;
plot3(Body_T(1:10:endT,1),Body_T(1:10:endT,2),Body_T(1:10:endT,3),'r.'); 
axis equal; hold on;
plot3(Cam_T(1:10:endT,1),Cam_T(1:10:endT,2),Cam_T(1:10:endT,3),'c.'); 
axis equal; hold on;
xlabel('x'); ylabel('y'); zlabel('z'); 
plot3(Body_T(1,1),Body_T(1,2),Body_T(1,3),'b.','MarkerSize',12);
plot3(Body_T(1*endT/4,1),Body_T(1*endT/4,2),Body_T(1*endT/4,3),'m.','MarkerSize',12);
plot3(Body_T(2*endT/4,1),Body_T(2*endT/4,2),Body_T(2*endT/4,3),'m.','MarkerSize',12);
plot3(Body_T(3*endT/4,1),Body_T(3*endT/4,2),Body_T(3*endT/4,3),'m.','MarkerSize',12);
plot3(Body_T(endT,1),Body_T(endT,2),Body_T(endT,3),'g.','MarkerSize',12);
title('body + cam0');

%% Read time stamps camera
% Tabcam = readtable('C:\Users\Seokju\Desktop\EuRoC-MAV\V1_01_easy\mav0\cam0\data.csv');
% Tabcam = readtable('C:\Users\Seokju\Desktop\EuRoC-MAV\MH_01_easy\mav0\cam0\data.csv');
Tabcam = readtable('C:\Users\Seokju\Desktop\EuRoC-MAV\MH_01_easy\mav0\cam1\data.csv');
TableTsC = Tabcam(:,1);
TableImageName = Tabcam(:,2);
ImNameArray = table2array(TableImageName);
TsCArray = table2array(TableTsC);

%% Timestamps synchronize
tic;
CPoseSync = {};
ImNameSync = {};
flag = 1;
for i =1:length(TsCArray)
    CurrentCamTime = TsCArray(i);
    DiffTime = abs(CurrentCamTime - TsSArray);
    [v ind] = min(DiffTime);
    if v < 999
        CPoseSync{flag} = MC{ind};
        ImNameSync{flag} = cell2mat(ImNameArray(i));
        flag = flag + 1;
    end
end

%Relative pose between sucessive images
% MRel = {}; a = 1;
% for i=2:length(CPoseSync)
%     MRel{a} = (CPoseSync{i-1})*inv(CPoseSync{i});
% %     MRel{a} = inv(CPoseSync{i-1})*(CPoseSync{i});
% %     MRel{a} = (CPoseSync{i})*inv(CPoseSync{i-1});
% %     MRel{a} = inv(CPoseSync{i})*(CPoseSync{i-1});
% %     MRel{a} = inv((CPoseSync{i})*inv(CPoseSync{i-1}));
% %     MRel{a} = inv(inv(CPoseSync{i})*(CPoseSync{i-1}));
% %     MRel{a} = inv((CPoseSync{i-1})*inv(CPoseSync{i}));
% %     MRel{a} = inv(inv(CPoseSync{i-1})*(CPoseSync{i}));
%     a = a + 1;
% end
elapsed = toc;
fprintf('<2> elapsed time: %.2f sec\n',elapsed);
        
%% intrinsic parameters of the camera
K = [458.654 0 367.215; 0 457.296 248.375; 0 0 1];
dist =  [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05];

%% Read the images
% Im_path = 'C:\Users\Seokju\Desktop\EuRoC-MAV\V1_01_easy\mav0\cam0\data\';
% Im_path = 'C:\Users\Seokju\Desktop\EuRoC-MAV\MH_01_easy\mav0\cam0\data\';
Im_path = 'C:\Users\Seokju\Desktop\EuRoC-MAV\MH_01_easy\mav0\cam1\data\';
Imagename = cell2mat(ImNameSync(1));
path = [Im_path Imagename];
TempIm = imread(path);

%% Undistord
[H W C]=size(TempIm);
f_new = 480; u0_new = W/2; v0_new = H/2;
K_new = [f_new 0 u0_new; 0 f_new v0_new; 0 0 1];
% Prev_Im = RemoveDisto(double(Prev_Im), dist, K, K_new);
% Prev_Im = uint8(Prev_Im);

%% Matching test
tic;
gap = 5;
outputs = [];
for i=1320:10:length(ImNameSync)-1
% for i=900:10:length(ImNameSync)-1
    
    %Read current image
    CurrPath = [Im_path cell2mat(ImNameSync(i))];
    CurrIm = imread(CurrPath);
    
    %Read next image
    NextPath = [Im_path cell2mat(ImNameSync(i+gap))];
    NextIm = imread(NextPath);
    
    %Undistort
    CurrIm = uint8(RemoveDisto(double(CurrIm), dist, K, K_new));
    NextIm = uint8(RemoveDisto(double(NextIm), dist, K, K_new));
    
    % Compute Fundamental matrix
    [F InMatch1 InMatch2] = compute_Fundamental_Mat(CurrIm,NextIm,4000);

    % Compute camera relative pose
%     CRelPos = (CPoseSync{i+gap})*inv(CPoseSync{i})
%     CRelPos = (CPoseSync{i})*inv(CPoseSync{i+gap})
    CRelPos = inv(CPoseSync{i+gap})*(CPoseSync{i})
%     CRelPos = inv(CPoseSync{i})*(CPoseSync{i+gap})

    %Find fundamental matrix from pose
    CRelR = CRelPos(1:3,1:3);
    CRelT = CRelPos(1:3,4);
    FCur = inv(K_new)'*skew(CRelT)*CRelR*inv(K_new);

    % Test the validity of points using Sampson
    Pts2 = [InMatch2.Location ones(length(InMatch2.Location),1)];
    Pts1 = [InMatch1.Location ones(length(InMatch1.Location),1)];
    d = diag(Pts2*FCur*Pts1'); d = d.^2;
    V1 = (FCur'*Pts2')'; V2 =(FCur*Pts1')';
    Err = d./(V1(:,1).^2 +V1(:,2).^2 + V2(:,1).^2 +V2(:,2).^2);
    fprintf('%d \t %.2f\t %.2f \n',size(Err,1),median(Err),mean(Err));
    elapsed = toc;
    fprintf('<3> elapsed time: %.2f sec\n',elapsed);

    outputs = [outputs; median(Err); mean(Err)];
    
    if(i>1800)
        break;
    end
    
end



