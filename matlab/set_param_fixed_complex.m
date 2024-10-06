

Ts = 0.01;
proximity_to_next_path = 10;
% initial uav conditions 
p0_line = [400;0;-10];
psi0_line = 0 ;

gamma_limit_line = pi/6;
phi_limit_line = pi/6;
kphi = 1;

k1_line = 1;
k2_line = 10;
k1_orb = 1;
k2_orb = 10;

V_line = 11;

%escolher o path
path = 1; %0 - quadrado, 1 - dubins , 2 - oito, 3 - duas orbits verticais, 4- reta de teste captura


%definicao dos paths a seguir hardcoded, definir cada um deles e ter
%atencao se a trajetoria faz sentido, adicionar aos vetores do ParamFixComplex

if(path == 0)
    %reference straight line 1
    c0_line1 = [0;0;-20];
    psi_l_line1 =0;  %desired heading angle
    gamma_l_line1 = 0; %desired flight path angle


    %reference straight line 2
    c0_line2 = [250;0;-20];
    psi_l_line2 =pi/2;  %desired heading angle
    gamma_l_line2 = 0; %desired flight path angle

    %reference straight line 3
    c0_line3 = [250;250;-20];
    psi_l_line3 = pi;  %desired heading angle
    gamma_l_line3 = 0; %desired flight path angle


     %reference straight line 4
    c0_line4 = [0;250;-20];
    psi_l_line4 = 3*pi/2;  %desired heading angle
    gamma_l_line4 = 0; %desired flight path angle



    % put parameters into structure
    ParamFixComplex.Ts = Ts;
    ParamFixComplex.proximity_to_next_path = proximity_to_next_path;
    %uav conditions
    ParamFixComplex.p0 = p0_line;
    ParamFixComplex.psi0 = psi0_line;

    ParamFixComplex.gamma_limit = gamma_limit_line;
    ParamFixComplex.phi_limit = phi_limit_line;
    ParamFixComplex.kphi = kphi;

    ParamFixComplex.k1_line = k1_line;
    ParamFixComplex.k2_line = k2_line;
    ParamFixComplex.k1_orb = k1_orb;
    ParamFixComplex.k2_orb = k2_orb;

    ParamFixComplex.V = V_line;
    %number of path segments
    ParamFixComplex.paths = [0, 0, 0, 0]; %0 - straight, 1 - orbit




    %path reference params

    ParamFixComplex.c0 = [c0_line1, c0_line2, c0_line3, c0_line4];

    %straight reference arrays

    ParamFixComplex.psi_l = [psi_l_line1, psi_l_line2, psi_l_line3, psi_l_line4];
    ParamFixComplex.gamma_l = [gamma_l_line1, gamma_l_line2, gamma_l_line3, gamma_l_line4];


    %orbit reference arrays


    ParamFixComplex.Rh = [0,0,0,0];
    ParamFixComplex.lambda = [0,0,0,0];
    ParamFixComplex.gamma_h = [0,0,0,0];
    ParamFixComplex.psi_h = [0,0,0,0];


end



if(path == 1)
     %dubins path

%     %segment line 1
%     c0_line1 = [50;20;-10];
%     psi_l_line1 = 0;  %desired heading angle
%     gamma_l_line1 = 0; %desired flight path angle

    %segment line 1
    c0_line1 = [250;20;-10];
    psi_l_line1 = 0;  %desired heading angle
    gamma_l_line1 = 0; %desired flight path angle


    %segment orb 2

     Rh_orb2 = 100;
     lambda_orb2 = 1;
     gamma_h_orb2 = 0;
     psi_h_orb2 = 3*pi/2;
     ch_orb2 = [500; 120; -10];

     %segment line 3
    c0_line3 = [500;220;-10];
    psi_l_line3 = pi;  %desired heading angle
    gamma_l_line3 = 0; %desired flight path angle


    %segment orb 4

     Rh_orb4 = 100;
     lambda_orb4 = 1;
     gamma_h_orb4 = 0;
     psi_h_orb4 = pi/2;
     ch_orb4 = [50; 120; -10];


    %segment line 5
    c0_line5 = [50;20;-10];
    psi_l_line5 = 0;  %desired heading angle
    gamma_l_line5 = 0; %desired flight path angle



    % put parameters into structure
    ParamFixComplex.Ts = Ts;   
    ParamFixComplex.proximity_to_next_path = proximity_to_next_path;
    %uav conditions
    ParamFixComplex.p0 = p0_line;
    ParamFixComplex.psi0 = psi0_line;

    ParamFixComplex.gamma_limit = gamma_limit_line;
    ParamFixComplex.phi_limit = phi_limit_line;
    ParamFixComplex.kphi = kphi;

    ParamFixComplex.k1_line = k1_line;
    ParamFixComplex.k2_line = k2_line;
    ParamFixComplex.k1_orb = k1_orb;
    ParamFixComplex.k2_orb = k2_orb;

    ParamFixComplex.V = V_line;

    ParamFixComplex.paths = [0, 1, 0, 1, 0]; %0 - straight, 1 - orbit




    %path reference params

    ParamFixComplex.c0 = [c0_line1, ch_orb2, c0_line3, ch_orb4, c0_line5];

    %straight reference arrays

    ParamFixComplex.psi_l = [psi_l_line1, 0, psi_l_line3, 0, psi_l_line5];
    ParamFixComplex.gamma_l = [gamma_l_line1, 0, gamma_l_line3, 0, gamma_l_line5];


    %orbit reference arrays


    ParamFixComplex.Rh = [0,Rh_orb2,0,Rh_orb4,0];
    ParamFixComplex.lambda = [0,lambda_orb2,0,lambda_orb4,0];
    ParamFixComplex.gamma_h = [0,gamma_h_orb2,0,gamma_h_orb4,0];
    ParamFixComplex.psi_h = [0,psi_h_orb2,0,psi_h_orb4,0];

end

% 
% if(path == 1)
%      %dubins path
% 
%     %segment line 1
%     c0_line1 = [50;20;-10];
%     psi_l_line1 = 0;  %desired heading angle
%     gamma_l_line1 = 0; %desired flight path angle
% 
% 
%     %segment orb 2
% 
%      Rh_orb2 = 100;
%      lambda_orb2 = 1;
%      gamma_h_orb2 = 0;
%      psi_h_orb2 = 3*pi/2;
%      ch_orb2 = [500; 120; -10];
% 
%      %segment line 3
%     c0_line3 = [500;220;-10];
%     psi_l_line3 = pi;  %desired heading angle
%     gamma_l_line3 = 0; %desired flight path angle
% 
% 
%     %segment orb 4
% 
%      Rh_orb4 = 100;
%      lambda_orb4 = 1;
%      gamma_h_orb4 = 0;
%      psi_h_orb4 = pi/2;
%      ch_orb4 = [50; 120; -10];
% 
% 
%     %segment line 5
%     c0_line5 = [50;20;-10];
%     psi_l_line5 = -pi/2;  %desired heading angle
%     gamma_l_line5 = 0; %desired flight path angle
% 
% 
% 
%     % put parameters into structure
%     ParamFixComplex.Ts = Ts;   
%     ParamFixComplex.proximity_to_next_path = proximity_to_next_path;
%     %uav conditions
%     ParamFixComplex.p0 = p0_line;
%     ParamFixComplex.psi0 = psi0_line;
% 
%     ParamFixComplex.gamma_limit = gamma_limit_line;
%     ParamFixComplex.phi_limit = phi_limit_line;
%     ParamFixComplex.kphi = kphi;
% 
%     ParamFixComplex.k1_line = k1_line;
%     ParamFixComplex.k2_line = k2_line;
%     ParamFixComplex.k1_orb = k1_orb;
%     ParamFixComplex.k2_orb = k2_orb;
% 
%     ParamFixComplex.V = V_line;
% 
%     ParamFixComplex.paths = [0, 1, 0, 1, 0]; %0 - straight, 1 - orbit
% 
% 
% 
% 
%     %path reference params
% 
%     ParamFixComplex.c0 = [c0_line1, ch_orb2, c0_line3, ch_orb4, c0_line5];
% 
%     %straight reference arrays
% 
%     ParamFixComplex.psi_l = [psi_l_line1, 0, psi_l_line3, 0, psi_l_line5];
%     ParamFixComplex.gamma_l = [gamma_l_line1, 0, gamma_l_line3, 0, gamma_l_line5];
% 
% 
%     %orbit reference arrays
% 
% 
%     ParamFixComplex.Rh = [0,Rh_orb2,0,Rh_orb4,0];
%     ParamFixComplex.lambda = [0,lambda_orb2,0,lambda_orb4,0];
%     ParamFixComplex.gamma_h = [0,gamma_h_orb2,0,gamma_h_orb4,0];
%     ParamFixComplex.psi_h = [0,psi_h_orb2,0,psi_h_orb4,0];
% 
% end


if(path == 2)

psi0_line = 3*pi/2 ;
    %segment orb 1

     Rh_orb1 = 50;
     lambda_orb1 = 1;
     gamma_h_orb1 = 0;
     psi_h_orb1 = 3*pi/2;
     ch_orb1 = [45; 0; 0];

    %segment orb 2

     Rh_orb2 = 50;
     lambda_orb2 = -1;
     gamma_h_orb2 = 0;
     psi_h_orb2 = 3*pi/2;
     ch_orb2 = [45; 95; 0];




    % put parameters into structure
    ParamFixComplex.Ts = Ts;
    ParamFixComplex.proximity_to_next_path = proximity_to_next_path;
    %uav conditions
    ParamFixComplex.p0 = p0_line;
    ParamFixComplex.psi0 = psi0_line;

    ParamFixComplex.gamma_limit = gamma_limit_line;
    ParamFixComplex.phi_limit = phi_limit_line;
    ParamFixComplex.kphi = kphi;

    ParamFixComplex.k1_line = k1_line;
    ParamFixComplex.k2_line = k2_line;
    ParamFixComplex.k1_orb = k1_orb;
    ParamFixComplex.k2_orb = k2_orb;

    ParamFixComplex.V = V_line;

    ParamFixComplex.paths = [1, 1];




    %path reference params

    ParamFixComplex.c0 = [ch_orb1, ch_orb2];

    %straight reference arrays

    ParamFixComplex.psi_l = [0, 0, 0, 0];
    ParamFixComplex.gamma_l = [0, 0, 0, 0];


    %orbit reference arrays


    ParamFixComplex.Rh = [Rh_orb1,Rh_orb2];
    ParamFixComplex.lambda = [lambda_orb1,lambda_orb2];
    ParamFixComplex.gamma_h = [gamma_h_orb1,gamma_h_orb2];
    ParamFixComplex.psi_h = [psi_h_orb1,psi_h_orb2];

end



if(path == 3)

psi0_line = 3*pi/2 ;
    %segment orb 1

     Rh_orb1 = 50;
     lambda_orb1 = -1;
     gamma_h_orb1 = -0.1;
     psi_h_orb1 = 3*pi/2;
     ch_orb1 = [10; 0; 0];

    %segment orb 2

     Rh_orb2 = 50;
     lambda_orb2 = -1;
     gamma_h_orb2 = 0.1;
     psi_h_orb2 = 3*pi/2;
     ch_orb2 = [0; 0; 150];




    % put parameters into structure
    ParamFixComplex.Ts = Ts;
    ParamFixComplex.proximity_to_next_path = proximity_to_next_path;
    %uav conditions
    ParamFixComplex.p0 = p0_line;
    ParamFixComplex.psi0 = psi0_line;

    ParamFixComplex.gamma_limit = gamma_limit_line;
    ParamFixComplex.phi_limit = phi_limit_line;
    ParamFixComplex.kphi = kphi;

    ParamFixComplex.k1_line = k1_line;
    ParamFixComplex.k2_line = k2_line;
    ParamFixComplex.k1_orb = k1_orb;
    ParamFixComplex.k2_orb = k2_orb;

    ParamFixComplex.V = V_line;

    ParamFixComplex.paths = [1, 1];




    %path reference params

    ParamFixComplex.c0 = [ch_orb1, ch_orb2];

    %straight reference arrays

    ParamFixComplex.psi_l = [0, 0, 0, 0];
    ParamFixComplex.gamma_l = [0, 0, 0, 0];


    %orbit reference arrays


    ParamFixComplex.Rh = [Rh_orb1,Rh_orb2];
    ParamFixComplex.lambda = [lambda_orb1,lambda_orb2];
    ParamFixComplex.gamma_h = [gamma_h_orb1,gamma_h_orb2];
    ParamFixComplex.psi_h = [psi_h_orb1,psi_h_orb2];

end


if(path == 4)
     %segment line 1
    c0_line1 = [50;20;-10];
    psi_l_line1 = 0;  %desired heading angle
    gamma_l_line1 = 0; %desired flight path angle




    % put parameters into structure
    ParamFixComplex.Ts = Ts;
    ParamFixComplex.proximity_to_next_path = proximity_to_next_path;
    %uav conditions
    ParamFixComplex.p0 = p0_line;
    ParamFixComplex.psi0 = psi0_line;

    ParamFixComplex.gamma_limit = gamma_limit_line;
    ParamFixComplex.phi_limit = phi_limit_line;
    ParamFixComplex.kphi = kphi;

    ParamFixComplex.k1_line = k1_line;
    ParamFixComplex.k2_line = k2_line;
    ParamFixComplex.k1_orb = k1_orb;
    ParamFixComplex.k2_orb = k2_orb;

    ParamFixComplex.V = V_line;
    %number of path segments
    ParamFixComplex.paths = [0]; %0 - straight, 1 - orbit




    %path reference params

    ParamFixComplex.c0 = [c0_line1];

    %straight reference arrays

    ParamFixComplex.psi_l = [psi_l_line1];
    ParamFixComplex.gamma_l = [gamma_l_line1];


    %orbit reference arrays


    ParamFixComplex.Rh = [0,0,0,0];
    ParamFixComplex.lambda = [0,0,0,0];
    ParamFixComplex.gamma_h = [0,0,0,0];
    ParamFixComplex.psi_h = [0,0,0,0];


end





