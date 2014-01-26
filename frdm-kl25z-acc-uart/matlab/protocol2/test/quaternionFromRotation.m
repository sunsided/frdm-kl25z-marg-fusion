function [q] = quaternionFromRotation(R)

    % http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

    %{
    m00 = R(1,1);    m01 = R(1,2);    m02 = R(1,3);
    m10 = R(2,1);    m11 = R(2,2);    m12 = R(2,3);
    m20 = R(3,1);    m21 = R(3,2);    m22 = R(3,3);
    
    trace = m00 + m11 + m22;

    if trace > 0
      S = sqrt(trace+1.0) * 2;              % S=4*qw 
      qw = 0.25 * S;
      qx = (m21 - m12) / S;
      qy = (m02 - m20) / S; 
      qz = (m10 - m01) / S; 
    elseif ((m00 > m11) && (m00 > m22))
      S = sqrt(1.0 + m00 - m11 - m22) * 2;  % S=4*qx 
      qw = (m21 - m12) / S;
      qx = 0.25 * S;
      qy = (m01 + m10) / S; 
      qz = (m02 + m20) / S; 
    elseif (m11 > m22)
      S = sqrt(1.0 + m11 - m00 - m22) * 2;  % S=4*qy
      qw = (m02 - m20) / S;
      qx = (m01 + m10) / S; 
      qy = 0.25 * S;
      qz = (m12 + m21) / S; 
    else
      S = sqrt(1.0 + m22 - m00 - m11) * 2;  % S=4*qz
      qw = (m10 - m01) / S;
      qx = (m02 + m20) / S;
      qy = (m12 + m21) / S;
      qz = 0.25 * S;
    end
    %}
    
    % http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    
    %{
    % "Angel" code
    
    trace = R(1,1) + R(2,2) + R(3,3);
    if( trace > 0 )
        s = 0.5 / sqrt(trace + 1.0);
        qw = 0.25 / s;
        qx = ( R(3,2) - R(2,3) ) * s;
        qy = ( R(1,3) - R(3,1) ) * s;
        qz = ( R(2,1) - R(1,2) ) * s;
    else
        if ( R(1,1) > R(2,2) && R(1,1) > R(3,3) )
            s = 2.0 * sqrt( 1.0 + R(1,1) - R(2,2) - R(3,3));
            qw = (R(3,2) - R(2,3) ) / s;
            qx = 0.25 * s;
            qy = (R(1,2) + R(2,1) ) / s;
            qz = (R(1,3) + R(3,1) ) / s;
        elseif (R(2,2) > R(3,3))
            s = 2.0 * sqrt( 1.0 + R(2,2) - R(1,1) - R(3,3));
            qw = (R(1,3) - R(3,1) ) / s;
            qx = (R(1,2) + R(2,1) ) / s;
            qy = 0.25 * s;
            qz = (R(2,3) + R(3,2) ) / s;
        else
            s = 2.0 * sqrt( 1.0 + R(3,3) - R(1,1) - R(2,2) );
            qw = (R(2,1) - R(1,2) ) / s;
            qx = (R(1,3) + R(3,1) ) / s;
            qy = (R(2,3) + R(3,2) ) / s;
            qz = 0.25 * s;
        end
    end
    %}
    
    % Alternative method
    
    m00 = R(1,1);    m01 = R(1,2);    m02 = R(1,3);
    m10 = R(2,1);    m11 = R(2,2);    m12 = R(2,3);
    m20 = R(3,1);    m21 = R(3,2);    m22 = R(3,3);

    qw = sqrt( max( 0, 1 + m00 + m11 + m22 ) ) / 2;
    qx = sqrt( max( 0, 1 + m00 - m11 - m22 ) ) / 2;
    qy = sqrt( max( 0, 1 - m00 + m11 - m22 ) ) / 2;
    qz = sqrt( max( 0, 1 - m00 - m11 + m22 ) ) / 2;
    
    % patch the signs
    qx = copysign( qx, m21 - m12 );
    qy = copysign( qy, m02 - m20 );
    qz = copysign( qz, m10 - m01 );
    
    q = [qw qx qy qz];
    
    function c = copysign(a, b)
        c = abs(a) * sign(b); % note that sign returns 0 if b is 0
    end
end