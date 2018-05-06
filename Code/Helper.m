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

%% Display Images and 3D Points
% Note this needs the computer vision toolbox.
figure,
subplot 121
imshow(I);
title('RGB Input Image');
subplot 122
imagesc(ID);
title('Depth Input Image');

figure,
pcshow(Pts,[r g b]/255);
drawnow;
title('3D Point Cloud');

ransacRuns = 3;
for i = 1:ransacRuns
    thld = 8;
    [test, testout] = ransac_point_cloud(Pts);
    
    
    MeanPt = mean(testout,2);
    Diff = sum(bsxfun(@minus,testout,MeanPt).^2);
    MeanPtModel = mean(Mdata,2);
    DiffModel = sum(bsxfun(@minus,Mdata,MeanPtModel).^2);
    Thld = max(DiffModel).*1.2;
    testout = testout(:,Diff<=Thld);
    
    
    
    
    hold on;
    plot(test{1});
    hold off;

    newPts = Pts;
    newPts(test{2}, :) = [];

    figure(),
    pcshow(newPts);
    drawnow;
    title(['3D Point Cloud after ', num2str(i), ' RANSAC runs for plane']);
    Pts = newPts;
end



