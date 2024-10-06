function [res] = sat(angle, angle_limit)
%SAT Summary of this function goes here
%   Detailed explanation goes here

if angle >= angle_limit
    res = angle_limit;
elseif angle <= -angle_limit
    res = -angle_limit;
else 
    res = angle;
end    

end

