function [c] = quaternionMul(a, b)

    as = a(1);    ax = a(2);    ay = a(3);    az = a(4);
    bs = b(1);    bx = b(2);    by = b(3);    bz = b(4);

    % http://willperone.net/Code/quaternion.php
    cs = as*bs - (ax*bx + ay*by + az*bz);
    cx = as*bx + ax*bs + ay*bz - az*by;
    cy = as*by + ay*bs + az*bx - ax*bz;
    cz = as*bz + az*bs + ax*by - ay*bx;
   
    c = [cs cx cy cz];
    
end