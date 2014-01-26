function [R] = quaternionToRotation(q)

    % Converts a quaternion to a right-handed 
    % (i.e. pre-multiplied) rotation matrix

    s = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    % http://willperone.net/Code/quaternion.php
    
    R = [1-2*(y*y+z*z), 2*(x*y-s*z),   2*(x*z+s*y);
         2*(x*y+s*z),   1-2*(x*x+z*z), 2*(y*z-s*x);
         2*(x*z-s*y),   2*(y*z+s*x),   1-2*(x*x+y*y)];

end