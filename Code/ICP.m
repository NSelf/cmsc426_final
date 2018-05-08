% template for ICP

function [R, t] = ICP(s, d, maxIters)
    thresh = .001;
    R = [1, 0, 0; 0, 1, 0; 0, 0, 1];
    t = [0, 0, 0];
    
    i = 1;
    dist = inf;
    
    snorm = pcnormals(s);
    dnorm = pcnormals(d);
    
    while i < maxIters && dist > thresh
        % mapping = find point to point correspondence
        % dist = mean distance between corresponding points
        %[R(i), t(i)] = rigid transform
        
       %s = R(i) * s prime + t(i)
       R = R(i) * R
       t = R(i) * t(i) + t
    end
    return R, t;
end