%% CMSC 426: Project 5 Helper Code
% Written by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)
% PhD in CS Student at University of Maryland, College Park
% Acknowledgements: Bhoram Lee of University of Pennsylvania for help with depthToCloud function

clc
clear all
close all



Path = '../Data/SingleObject/'; 
SceneNum = 1; %0, 1, 2, 6, 8, 12, 22, 23
SceneName = sprintf('%0.3d', SceneNum);
numFrames = ((size(dir([Path,'scene_', SceneName, '/frames']), 1) - 2)/2);
foregrounds = cell(numFrames, 1);
for fr = 1:5 %numFrames
    close all;
    
    %% Read RGB and Depth Images

    FrameNum = num2str(fr - 1); % Frames are 0-indexed, doesn't play well 
                                % w/ MATLAB arrays

    I = imread([Path,'scene_',SceneName,'/frames/frame_',FrameNum,'_rgb.png']);
    ID = imread([Path,'scene_',SceneName,'/frames/frame_',FrameNum,'_depth.png']);

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
    % X,Y              - registered x and y image (NaN for invalid pixel)`
    % validInd	   - indices of pixels that are not NaN or zero
    % NOTE:
    % - pcz equals to D_(validInd)
    % - pcx equals to X(validInd)
    % - pcy equals to Y(validInd)

    [pcx, pcy, pcz, r, g, b, D_, X, Y,validInd] = depthToCloud_full_RGB(ID, I, './params/calib_xtion.mat');
    pts = [pcx pcy pcz];

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
    pcshow(pts,[r g b]/255);
    drawnow;
    title('3D Point Cloud');

    %% Noise exclusion by radius from center of foreground
    figure,
    toDel = [];
    center = [0, 0, 0];
    radius = 1200;

    for i = 1:length(pts)
        dist = norm(pts(i, :) - center);
        if dist > radius
            toDel = [toDel; i];
        end
    end

    newPts = pts;
    newPts(toDel, :) = [];
    r(toDel) = [];
    g(toDel) = [];
    b(toDel) = [];

    pcshow(newPts, [r g b]/255);
    title(['3D Point Cloud after restiction of radius = ', num2str(radius), ' from Origin.']);
    pts = newPts;

    %% RANSAC for background plane removal
    toDel = [];
    ransacRuns = 2;
    for i = 1:ransacRuns
        toDel = ransac_point_cloud(pts);
        figure,
        pcshow(newPts);
        hold on;
        plot(toDel{1});
        hold off;

        newPts = pts;
        newPts(toDel{2}, :) = [];

        figure,
        pcshow(newPts);
        title(['3D Point Cloud after ', num2str(i), ' RANSAC runs for plane']);
        pts = newPts;
    end


    %% Avg distance noise suppression
    figure,
    toDel = [];
    center = mean(pts);
    hold on;
    plot3(center(1), center(2), center(3), 'xk');
    dist = zeros(length(pts), 1);
    for i = 1:length(pts)
        dist(i) = norm(pts(i, :) - center);
    end

    s = std(dist);
    numStds = 2.25;

    for i = 1:length(pts)
        if dist(i) > numStds*s
            toDel = [toDel; i];
        end
    end

    newPts = pts;
    newPts(toDel, :) = [];
    r(toDel) = [];
    g(toDel) = [];
    b(toDel) = [];

    pcshow(newPts);
    title(['3D Point Cloud after suppressing noise >', numStds, '*std distance from mean location of remaining points.']);
    pts = newPts;
    
    foregrounds{fr} = pts;

    if (fr > 1)
        %% ICP 
        maxIters = 10;
        A = foregrounds{fr-1};
        B = foregrounds{fr};
        A_prime = foregrounds{fr-1};
        B_prime = foregrounds{fr};
        [R, t] = ICP(A, B, maxIters, 0.5);

        %% Morph objects
        A_prime = pctransform(A, affine3d(t));
        B_prime = [A_prime; B];

        figure
        pcshow(B_prime);
%         for i = 1:size(A, 1)
%             A_prime(i,:) = t'*(R*A(i,:)');
%         end
%         B_prime = [A_prime;B];
%         figure,
%         pcshow(B_prime);
        title(['Merge of ', fr-1, 'and ', fr, ' point clouds.']);
    end
end