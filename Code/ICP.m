function [R, T] = ICP(A, B, maxiter, thres)
    
    T = zeros(3,1);
    R = eye(3,3);
    
    e = 0.001;
    dist = 0.011;
    k = 1;
    
    
    while (k < maxiter)
        M = KDTreeSearcher(A);
        if(size(A,1) > size(B,1))
            A = A(1:size(B,1), :);
        else
            B = B(1:size(A,1), :);
        end

        if(size(B,1) == 3)
            pc = pointCloud(B');
        else
            pc = pointCloud(B);
        end
        norm = pcnormals(pc);
        [Ri, Ti] = Point_To_Plane(A,B,norm);
        if(size(A,1) ~= 3)
            M1 = Ri * A';
            A = M1 + Ti;
            R = Ri * R;
            T = Ri * Ti + T;
            k = k + 1;
            B = B';
        else
            M1 = Ri * A;
            A = M1 + Ti;
            R = Ri * R;
            T = Ri * Ti + T;
            k = k + 1;
        end
    end
end
       