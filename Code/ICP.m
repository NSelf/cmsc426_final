function [R, T] = ICP(p1, p2, method, varargin)

if(nargin==2)
    % Default is point to point ICP
    method = 'PointToPoint';
end

if(strcmp(method,'PointToPoint'))
    p1Mean = mean(p1,2);
    p2Mean = mean(p2,2);
    
    % Compute q, which is the mean subtracted values
    q1 = bsxfun(@minus, p1, p1Mean);
    q2 = bsxfun(@minus, p2, p2Mean);
    
    % Compute H matrix
    H = q1*q2';
    
    % Compute SVD
    [U, ~, V] = svd(H);
    
    % Compute X
    X = V*U';
    R = X;
    
    T = p2Mean-R*p1Mean;
    return;
end

if(strcmp(method,'PointToPlane'))
    Normals = varargin{1}; % Size 3XN
    Diff = p1-p2; % destination - source
    b = transpose(sum(Diff.*Normals,1)); 
    A = [Normals(3,:).*p2(2,:)-Normals(2,:).*p2(3,:);
         Normals(1,:).*p2(3,:)-Normals(3,:).*p2(1,:);
         Normals(2,:).*p2(1,:)-Normals(1,:).*p2(2,:);
         Normals]'; % NX6
    
    x = A\b;
    
    cosAlpha = cos(x(1));
    sinAlpha = sin(x(1));
    cosBeta = cos(x(2));
    sinBeta = sin(x(2));
    cosGamma = cos(x(3));
    sinGamma = sin(x(3));
        
    R = [cosGamma*cosBeta, -sinGamma*cosAlpha+cosGamma*sinBeta*sinAlpha, sinGamma*sinAlpha+cosGamma*sinBeta*cosAlpha;
         sinGamma*cosBeta, cosGamma*cosAlpha+sinGamma*sinBeta*sinAlpha, -cosGamma*sinAlpha+sinGamma*sinBeta*cosAlpha;
         -sinBeta,          cosBeta*sinAlpha,                            cosBeta*cosAlpha];
    T = x(4:end);
    return;
end
end