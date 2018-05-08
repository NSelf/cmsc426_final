%% To Test ICP

clc
clear all
close all

%% Setup Paths and Read RGB and Depth Images
Path = '../Data/SingleObject/'; 
SceneNum = 1;
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

B = Pts;

%% ICP test

maxIters = 0;

[R, t] = ICP(A, B, maxIters);


