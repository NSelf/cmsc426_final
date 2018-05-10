function [R, T] = ICP(A, B, maxiter, thres)
    
    T = zeros(3,1);
    R = eye(3,3);
    r = [0;0;0];
    
    e = 0.001;
    dist = 0.011;
    k = 1;
    
    totalDist = e + .0001;
    while (k < maxiter && totalDist > e)
        M = KDTreeSearcher(A);
        if(size(A,1) > size(B,1))
            A = A(1:size(B,1), :);
        else
            B = B(1:size(A,1), :);
        end

        pc = pointCloud(B);
        norm = pcnormals(pc);
        diff = A - B;
        for i = 1:size(diff, 1)
            d1 = dot(diff(i,:),norm(i,:));
            d2 = dot(T,norm(i,:)');
            c = cross(A,norm);
            d3 = dot(r,c(i,:));
            dist = d1 + d2' + d3;
            dist = dist^2;
            totalDist = totalDist + dist;
        end
        [Ri, Ti] = Point_To_Plane(A,B,norm);
%         if(size(A,1) ~= 3)
%             M1 = Ri * A';
%             A = M1 + Ti;
%             R = Ri * R;
%             T = Ri * Ti + T;
%             k = k + 1;
%             B = B';
%         else
            M1 = Ri * A';
            A = M1 + Ti;
            A = A';
            R = Ri * R;
            T = Ri * Ti + T;
            k = k + 1;
%         end
    end
end
       