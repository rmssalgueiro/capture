function [  gammac, phic, psid] = path_following_controller(u, psi, psid1, ParamFixComplex)

gamma_limit = ParamFixComplex.gamma_limit;
phi_limit = ParamFixComplex.phi_limit;
kphi = ParamFixComplex.kphi;

gammac = -sat(asin(u(3,1)/ParamFixComplex.V), gamma_limit);

psid = atan2(u(2,1),u(1,1));

while abs(psid - psid1) > 3*pi/2
    if((psid - psid1) > 3*pi/2) 
        psid = psid- 2*pi;
    elseif ((psid - psid1) < - 3*pi/2)
        psid = psid + 2*pi;
    end
end

%if(abs(psid - psid1) > pi)
%    xpto = 0;
%end;

phic = sat(kphi*(psid - psi), phi_limit);

% if isnan(gammac) || isnan(phic)
%     xpto = 0;
% end
%     

end
