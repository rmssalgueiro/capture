function [t0, xt, xs] = apply_controls(T, t0, f_t, xt,f_s,xs,u_shuttle, ParamFixComplex,path_segment, path_type)

persistent psid1;
if isempty(psid1)
        psid1 = 0;
end




st_target = xt;
% %straight line path generator
% p_target = [xt(1); xt(2); xt(3)];
% u_velocity_vector = straight_path_generator(p_target, ParamFixLine)
% 
% %straight line path following controller
% [gammac,phic] = straight_path_following_controller(u_velocity_vector, xt(4), ParamFixLine)
% 
% 

%complex path generator
u  = complex_path_generator(st_target(1:3), ParamFixComplex, path_segment, path_type);

%complex path following controller

[  gammac, phic, psid] = path_following_controller(u, st_target(4), psid1  , ParamFixComplex);

psid1 = psid;

%%achar as variaveis con_traget
con_target = [ParamFixComplex.V; gammac; phic];
f_t_value = f_t(st_target,con_target);
st_target = st_target + (T*f_t_value);
xt = full(st_target);




st_shuttle = xs(1:7,:);
f_s_value = f_s(st_shuttle,u_shuttle);
st_shuttle = st_shuttle + (T*f_s_value);
xs = full(st_shuttle);


t0 = t0 + T;

end
