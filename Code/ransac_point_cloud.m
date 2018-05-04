function [ planeInlierIndices ] = ransac_point_cloud(pointCloud)

minSample = 3;
threshold = 8;
maxRuns = 500;
maxInlierCount = 0;
bestMatch = cell(1, 2); % plane initial points, inlier indices
for i = 1:maxRuns
    sampleSet = pointCloud(randi([1, size(pointCloud, 1)], 1, minSample), :);

    inlierIndices = [];
    p1 = sampleSet(1, :);
    p2 = sampleSet(2, :);
    p3 = sampleSet(3, :);
    normal = cross(p1-p2, p1-p3);
    a = normal(1);
    b = normal(2);
    c = normal(3);
    d = -(a*p1(1) + b*p1(2) + c*p1(3));
    samplePlane = planeModel([a, b, c, d]);
    for j = 1:size(pointCloud, 1)
        x = pointCloud(j, 1);
        y = pointCloud(j, 2);
        z = pointCloud(j, 3);
        dist = (a*x+b*y+c*z+d) / sqrt(a^2+b^2+c^2);
        if (abs(dist) < threshold)
            inlierIndices = [inlierIndices; j];
        end
    end
    if (length(inlierIndices) > maxInlierCount)
        bestMatch{1} = samplePlane;
        bestMatch{2} = inlierIndices;
        maxInlierCount = length(inlierIndices);
    end
end

planeInlierIndices = bestMatch;

end