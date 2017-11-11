function [qout] = quaternion_inv(qin)
    
n = norm(qin);
qout(1) = qin(1)/n;
qout(2) = -qin(2)/n;
qout(3) = -qin(3)/n;
qout(4) = -qin(4)/n;


end