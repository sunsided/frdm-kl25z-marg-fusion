function [c] = quaternionConj(a)

    as = a(1);    ax = a(2);    ay = a(3);    az = a(4);
   
    c = [as -ax -ay -az];
    
end