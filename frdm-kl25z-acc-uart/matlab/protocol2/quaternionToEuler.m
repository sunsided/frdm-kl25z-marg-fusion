function [roll, pitch, yaw] = quaternionToEuler(q)

        qw = q(1);
        qx = q(2);
        qy = q(3);
        qz = q(4);

        rotateXa0 = 2.0*(qy*qz + qw*qx);
        rotateXa1 = qw*qw - qx*qx - qy*qy + qz*qz;
        rotateX = 0.0;
        if rotateXa0 ~= 0.0 && rotateXa1 ~= 0.0
            rotateX = atan2(rotateXa0, rotateXa1);
        end

        rotateYa0 = -2.0*(qx*qz - qw*qy);
        rotateY = 0.0;
        if rotateYa0 >= 1.0 
            rotateY = pi/2.0;
        elseif rotateYa0 <= -1.0 
            rotateY = -pi/2.0;
        else
            rotateY = asin(rotateYa0);
        end

        rotateZa0 = 2.0*(qx*qy + qw*qz);
        rotateZa1 = qw*qw + qx*qx - qy*qy - qz*qz;
        rotateZ = 0.0;
        
        if rotateZa0 ~= 0.0 && rotateZa1 ~= 0.0
            rotateZ = atan2(rotateZa0, rotateZa1);
        end

        roll = radtodeg(rotateX);
        pitch = radtodeg(rotateY);
        yaw = radtodeg(rotateZ);
        
end