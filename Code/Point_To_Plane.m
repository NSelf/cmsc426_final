function [R, T] = Point_To_Plane(p1, p2, norm)

    Nmls = norm;
    if(size(p1,1) ~= 3 & size(p2,1) ~= 3)
        m = min(size(p1,1),size(p2,1));
        if(size(p1,1) < size(p2,1))
            p2 = p2(1:m,:);
        else
            p1 = p1(1:m,:);
        end
        Nmls = Nmls(1:m,:);
        Nmls = Nmls'
        p1 = p1';
        p2 = p2';
    else
        Nmls = Nmls';
    end
    sub = p1-p2; 
    b = sum(sub.*Nmls,1)';
    
    A11 = Nmls(3,:).*p2(2,:);
    A12 = Nmls(2,:).*p2(3,:);
    A1 = A11 - A12;
    
    A21 = Nmls(1,:).*p2(3,:);
    A22 = Nmls(3,:).*p2(1,:);
    A2 = A21-A22;
    
    A31 = Nmls(2,:).*p2(1,:);
    A32 = Nmls(1,:).*p2(2,:);
    A3 = A31 - A32;
    A = [A1;A2;A3;Nmls]'; 
    
    x = A\b;
    
    one = x(1);
    two = x(2);
    three = x(3);
    
    aca = cos(one);
    asa = sin(one);
    bcb = cos(two);
    bsb = sin(two);
    gcg = cos(three);
    gsg = sin(three);
    
    a = gcg*bcb;
    b1 = -gsg*aca;
    b2 = gcg*bsb*asa;;
    b = b1+b2;
    c1 = gsg*asa; 
    c2 = gcg*bsb*aca;
    c = c1+c2;
    d = gsg*bcb;
    e1 = gcg*aca;
    e2 = gsg*bsb*asa;
    e = e1+e2;
    f1 = -gcg*asa;
    f2 = gsg*bsb*aca;
    f = f1+f2;
    g = -bsb;
    h = bcb*asa
    i = bcb*aca;
    
    R = [a, b, c; d, e, f; g, h, i];
    T = x(4:end);
    return;
end