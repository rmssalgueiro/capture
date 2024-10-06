function u  = complex_path_generator(p, ParamFixComplex, path_segment, path_type)



if path_type == 0                %straight line following
    %Variables

    psi_l = ParamFixComplex.psi_l(path_segment);  %desired heading angle
    gamma_l = ParamFixComplex.gamma_l(path_segment); %desired flight path angle

    c_n = ParamFixComplex.c0(1,path_segment);    %path starting point
    c_e = ParamFixComplex.c0(2,path_segment);
    c_d = ParamFixComplex.c0(3,path_segment);
    
    
    r_n = p(1);    %position
    r_e = p(2);
    r_d = p(3);
    r = [r_n; r_e; r_d];

    cl = [c_n; c_e; c_d];

    n_lon = [ -sin(psi_l) ; cos(psi_l) ; 0 ]; 

    n_lat = [-cos(psi_l)*sin(gamma_l) ; -sin(psi_l)*sin(gamma_l) ; -cos(gamma_l) ];

     k1 = ParamFixComplex.k1_line;
    k2 = ParamFixComplex.k2_line;


    %u_line = k1*(n_lon*transpose(n_lon) +  n_lat*transpose(n_lat))*(r - cl) + k2*cross(n_lon,n_lat);
    u_line = -k1*(n_lon*n_lon' +  n_lat*n_lat')*(r - cl) - k2*cross(n_lon,n_lat);

    u = ParamFixComplex.V*(u_line/norm(u_line));

% if  any(isnan(n_lon)) || any(isnan(n_lat))
%     xpto = 0;
% end
    
else                            %orbit following  
    %Variables

     Rh = ParamFixComplex.Rh(path_segment);
     lambda = ParamFixComplex.lambda(path_segment);
     gamma_h = ParamFixComplex.gamma_h(path_segment);
     psi_h = ParamFixComplex.psi_h(path_segment);

    k1 = ParamFixComplex.k1_orb;
    k2 = ParamFixComplex.k2_orb;



    c_n = ParamFixComplex.c0(1,path_segment);   %center of helix
    c_e = ParamFixComplex.c0(2,path_segment);
    c_d = ParamFixComplex.c0(3,path_segment);

     r_n = p(1);
     r_e = p(2);
     r_d = p(3);




    dalpha_cyl =  [ 2*(r_n - c_n)/Rh ; 2*(r_e - c_e)/Rh ; 0 ] ;

    dalpha_pl = [ (tan(gamma_h)/lambda)*(-(r_e - c_e))/( (r_n - c_n)^2 + (r_e - c_e)^2 ) ; (tan(gamma_h)/lambda)*(r_n - c_n)/( (r_n - c_n)^2 + (r_e - c_e)^2 )  ; 1/Rh ] ;


    alpha_cyl = ( (r_n - c_n)/Rh )^2 + ( (r_e - c_e)/Rh )^2 -1;

    alpha_pl = ( (r_d - c_d)/Rh )^2 + (tan(gamma_h)/lambda)*(atan( (r_e - c_e)/(r_n - c_n) ) - psi_h);

    u_line = k1*(-alpha_cyl*dalpha_cyl + alpha_pl*dalpha_pl) - lambda*k2*( (2/Rh)*[ (r_e - c_e)/Rh ; -(r_n - c_n)/Rh ; lambda*tan(gamma_h) ]) ;

    u =  ParamFixComplex.V*(u_line/norm(u_line));

% if  any(isnan(dalpha_cyl)) || any(isnan(dalpha_pl))
%     xpto = 0;
% end

end




% if any(isnan(u)) || any(isnan(u_line)) 
%     xpto = 0;
% end


end