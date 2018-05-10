function [R, T] = Point_To_Plane(p1, p2, norm)

    Normals = norm;
    if(size(p1,1) ~= 3 & size(p2,1) ~= 3)
        m = min(size(p1,1),size(p2,1));
        if(size(p1,1) < size(p2,1))
            p2 = p2(1:m,:);
        else
            p1 = p1(1:m,:);
        end
        Normals = Normals(1:m,:);
        Normals = Normals'
        p1 = p1';
        p2 = p2';
    else
        Normals = Normals';
    end
    Diff = p1-p2; 
    b = transpose(sum(Diff.*Normals,1)); 
    A = [Normals(3,:).*p2(2,:)-Normals(2,:).*p2(3,:);
         Normals(1,:).*p2(3,:)-Normals(3,:).*p2(1,:);
         Normals(2,:).*p2(1,:)-Normals(1,:).*p2(2,:);
         Normals]'; 
    
    x = A\b;
    
    cosA = cos(x(1));
    sinA = sin(x(1));
    cosB = cos(x(2));
    sinB = sin(x(2));
    cosG = cos(x(3));
    sinG = sin(x(3));
        
    R = [cosG*cosB, -sinG*cosA+cosG*sinB*sinA, sinG*sinA+cosG*sinB*cosA;
         sinG*cosB, cosG*cosA+sinG*sinB*sinA, -cosG*sinA+sinG*sinB*cosA;
         -sinB,cosB*sinA,cosB*cosA];
    T = x(4:end);
    return;
end