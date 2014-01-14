function [rp] = rollpitch(acc)
    % ROLLPITCH Calculates roll and pitch angles from measured acceleration
    %
    %   Parameters
    %   acc = 3D vector of component accelerations [x; y; z]
    %         where z is interpreted as up vector, that is, if the
    %         accelerometer measures 1g downwards, then z is positive.
    %         
    %
    %   Return values
    %   vector [roll; pitch] in radians

    % Get normalized component vectors
    invn = 1/norm(acc);
    x =  [1; 0; 0] * acc(1) * invn;
    y =  [0; 1; 0] * acc(2) * invn;

    % Reference vector
    zref = [0; 0; 1];

    % Prepare result
    rp = [NaN; NaN];
    
    % Get y-z component angle (roll)
    zy_cross = cross(zref, y);
    yangle   = zy_cross(1);
    rp(1)    = asin(yangle);
    if acc(3) > 0
       rp(1) = pi - rp(1);
    end
    
    % Get x-z component angle (pitch)
    xz_cross = cross(x, zref);
    xangle   = xz_cross(2);
    rp(2)     = asin(xangle);
    if acc(3) > 0
       rp(2) = -rp(2);
    end
end