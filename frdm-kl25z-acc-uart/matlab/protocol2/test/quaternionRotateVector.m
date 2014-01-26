function [v] = quaternionRotateVector(q, v)

    % build a vector quaternion
    V = [0 v(1) v(2) v(3)];
    
    % Calculate q*V* q^-1
    % Falls back to q*V*q' if q is a unit quaternion
    V = quaternionMul(quaternionMul(q, V), quaternionConj(q));
    
    % Extract vector component
    v = [V(2) V(3) V(4)];

end