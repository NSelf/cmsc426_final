%% CMSC 426: Project 5 Helper Code
% Written by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)
% PhD in CS Student at University of Maryland, College Park
% Acknowledgements: Bhoram Lee of University of Pennsylvania for help with depthToCloud function

clc
clear all
close all

%% Setup Paths and Read RGB and Depth Images
Path = '../SingleObject/'; 
SceneNum = 1;
SceneName = sprintf('%0.3d', SceneNum);
FrameNum = num2str(1);

I = imread([Path,'scene_',SceneName,'/frames/image_',FrameNum,'_rgb.png']);
ID = imread([Path,'scene_',SceneName,'/frames/image_',FrameNum,'_depth.png']);

%% Extract 3D Point cloud
% Inputs:
% ID - the depth image
% I - the RGB image
% calib_file - calibration data path (.mat) 
%              ex) './param/calib_xtion.mat'
%              
% Outputs:
% pcx, pcy, pcz    - point cloud (valid points only, in RGB camera coordinate frame)
% r,g,b            - color of {pcx, pcy, pcz}
% D_               - registered z image (NaN for invalid pixel) 
% X,Y              - registered x and y image (NaN for invalid pixel)
% validInd	   - indices of pixels that are not NaN or zero
% NOTE:
% - pcz equals to D_(validInd)
% - pcx equals to X(validInd)
% - pcy equals to Y(validInd)

[pcx, pcy, pcz, r, g, b, D_, X, Y,validInd] = depthToCloud_full_RGB(ID, I, './params/calib_xtion.mat');
Pts = [pcx pcy pcz];

%[pcx, pcy, pcz, r, g, b, D_, X, Y,validInd] = depthToCloud_full_RGB(ID, I, './params/calib_xtion.mat');
%Pts = transpose(s_cam3*[pcx pcy pcz]*[0 0 1;-1 0 0;0 -1 0]'*R_cv3 + repmat(t_cv3,length(pcx),1));

%% Display Images and 3D Points
% Note this needs the computer vision toolbox.

toDel = [];
center = [0, 0, 0];
radius = 1200;

for i = 1:length(Pts)
    dist = norm(Pts(i, :) - center);
    if dist > radius
        toDel = [toDel; i];
    end
end

newPts = Pts;
newPts(toDel, :) = [];
r(toDel) = [];
g(toDel) = [];
b(toDel) = [];

Pts = newPts;

%% RANSAC for background plane removal
toDel = [];
ransacRuns = 2;
for i = 1:ransacRuns
    toDel = ransac_point_cloud(Pts);
    hold on;
    plot(toDel{1});
    hold off;
    
    newPts = Pts;
    newPts(toDel{2}, :) = [];

    Pts = newPts;
end


%% Avg distance noise suppression
toDel = [];
center = mean(Pts);
dist = zeros(length(Pts), 1);
for i = 1:length(Pts)
    dist(i) = norm(Pts(i, :) - center);
end

std = std(dist);

for i = 1:length(Pts)
    if dist(i) > 2*std
        toDel = [toDel; i];
    end
end

newPts = Pts;
newPts(toDel, :) = [];
r(toDel) = [];
g(toDel) = [];
b(toDel) = [];

figure(),
pcshow(newPts);
drawnow;
Pts = newPts;

A = Pts;

%% Second Scene

SceneNum = 2;
SceneName = sprintf('%0.3d', SceneNum);
FrameNum = num2str(1);

I = imread([Path,'scene_',SceneName,'/frames/frame_',FrameNum,'_rgb.png']);
ID = imread([Path,'scene_',SceneName,'/frames/frame_',FrameNum,'_depth.png']);

%% Extract 3D Point cloud

[pcx, pcy, pcz, r, g, b, D_, X, Y,validInd] = depthToCloud_full_RGB(ID, I, './params/calib_xtion.mat');
Pts = [pcx pcy pcz];

%% Noise exclusion by radius from center of foreground
toDel = [];
center = [0, 0, 0];
radius = 1200;

for i = 1:length(Pts)
    dist = norm(Pts(i, :) - center);
    if dist > radius
        toDel = [toDel; i];
    end
end

newPts = Pts;
newPts(toDel, :) = [];
r(toDel) = [];
g(toDel) = [];
b(toDel) = [];

Pts = newPts;

%% RANSAC for background plane removal
toDel = [];
ransacRuns = 2;
for i = 1:ransacRuns
    toDel = ransac_point_cloud(Pts);
    hold on;
    plot(toDel{1});
    hold off;
    
    newPts = Pts;
    newPts(toDel{2}, :) = [];

    Pts = newPts;
end


%% Avg distance noise suppression
toDel = [];
center = mean(Pts);
dist = zeros(length(Pts), 1);
for i = 1:length(Pts)
    dist(i) = norm(Pts(i, :) - center);
end

%std = std(dist);

for i = 1:length(Pts)
    if dist(i) > 2*std
        toDel = [toDel; i];
    end
end

newPts = Pts;
newPts(toDel, :) = [];
r(toDel) = [];
g(toDel) = [];
b(toDel) = [];

figure(),
pcshow(newPts);
drawnow;
Pts = newPts;

B = Pts;

%% ICP test
A = load('A.mat');
B = load('B.mat');
A = A.A;
B = B.B;
maxIters = 10;

[R, t] = ICP(A, B, maxIters, 0.5);

