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
    A1 = Normals(3,:).*p2(2,:)-Normals(2,:).*p2(3,:);
    A2 = Normals(1,:).*p2(3,:)-Normals(3,:).*p2(1,:);
    A3 = Normals(2,:).*p2(1,:)-Normals(1,:).*p2(2,:);
    A = [A1;A2;A3;Normals]'; 
    
    x = A\b;
    
    cosa = cos(x(1));
    sina = sin(x(1));
    cosb = cos(x(2));
    sinb = sin(x(2));
    cosg = cos(x(3));
    sing = sin(x(3));
    
    a = cosg*cosb;
    b = -sing*cosa+cosg*sinb*sina;
    c = sing*sina+cosg*sinb*cosa;
    d = sing*cosb;
    e = cosg*cosa+sing*sinb*sina;
    f = -cosg*sina+sing*sinb*cosa;
    g = -sinb;
    h = cosb*sina
    i = cosb*cosa;
    
    R = [a, b, c;
         d, e, f;
         g, h, i];
    T = x(4:end);
    return;
end