
%clear all; clc; close all;


addpath('C:\Program Files\Polyspace\R2020a\bin\win64\casadi-3.6.3-windows64-matlab2018b')
addpath('D:\Docments\Matlab\toolbox\casadi-3.6.3-windows64-matlab2018b')
import casadi.*







% M690B drone MPC controller definition
% state
pos_shuttle = MX.sym('pos_shuttle',3);
vel_shuttle = MX.sym('vel_shuttle',3);
psi_shuttle = MX.sym('psi_shuttle',1);
pos_target = MX.sym('pos_target',3);
vel_target = MX.sym('vel_target',3);
psi_target = MX.sym('psi_target',1);
xx = [pos_shuttle;vel_shuttle;psi_shuttle; pos_target ;vel_target;psi_target];
n_states = size(xx,1);

% control
acc_shuttle = MX.sym('acc_shuttle',3);
omegaz_shuttle = MX.sym('omegaz_shuttle',1);
uu = [acc_shuttle;omegaz_shuttle];
n_controls = size(uu,1);
u_min = [-20; -20; -20; -pi]; u_max = [20; 20; 20; pi];

% Simplified Nonlinear ODE (Ordinary Differential Equation)
xdot = [vel_shuttle; acc_shuttle; omegaz_shuttle;vel_target;zeros(3,1); 0];
f_cont = Function('f_cont',{xx,uu},{xdot},{'x','u'},{'ode'}); % last two represent labels for better understanding

% controller parameters
Param.T = 5; % Time horizon
Param.N = 25; % Number of control intervals
Param.Q = blkdiag(8,8,6,1,1,0,1)*10;
Param.R = blkdiag(0.2,0.2,0.1,1)*10;
Param.S = blkdiag(1,1)*0;
%Param.S = blkdiag(1,1,1)*10;

Param.closeness = 1.5;
Param.height = 2;

% Integrator to discretize the system
intg_options = struct;
intg_options.tf = Param.T/Param.N;
intg_options.simplify = true;
intg_options.number_of_finite_elements = 4;
% DAE (Differential-algebraic equations) problem
dae = struct;
dae.x = xx;          % States
dae.p = uu;          % Parameters are the things thar are fixed during the integration horizon --> In this case, controls
dae.ode = f_cont(xx,uu);   % System right hand side f(x,u) = ode
intg = integrator('intg','rk', dae, intg_options); % Label, especific integration method, problem definition, options
result = intg('x0',xx,'p',uu); % Evaluate with symbols
x_next = result.xf;
f_disc = Function('f_disc',{xx,uu},{x_next},{'x','u'},{'x_next'}); % {x,u} -> {x_next}


% Optimal control problem using multiple-shooting
MPC = casadi.Opti(); % to use ipopt
% MPC = casadi.Opti('conic'); % to use qpoasis
Param.xx = MPC.variable(n_states,Param.N+1); % Decision variables for state trajectory
Param.uu = MPC.variable(n_controls,Param.N);
Param.xx0 = MPC.parameter(n_states,1); % Because MPC, the first state must be equal to a measurement --> Parameter (not optimized over)


Param.dZ = MPC.variable();
Param.distance = MPC.variable();

obj = 0;



for k = 1:Param.N
    
      Param.distance = sqrt((Param.xx(1,k)-Param.xx(8,k))^2 + (Param.xx(2,k)-Param.xx(9,k))^2  ) ;
  Param.dZ =  if_else( Param.distance <= Param.closeness,0 ,Param.height );
    
  %obj = obj + (Param.xx(1:7,k)-(Param.xx(8:14,k)-[0;0;Param.dZ+0.7;0;0;0;0]))'*Param.Q*(Param.xx(1:7,k)-(Param.xx(8:14,k)-[0;0;Param.dZ+0.7;0;0;0;0])) + Param.uu(:,k)'*Param.R*Param.uu(:,k) ...         
    obj = obj + (Param.xx(1:7,k)-(Param.xx(8:14,k)-[0;0;Param.dZ;0;0;0;0]))'*Param.Q*(Param.xx(1:7,k)-(Param.xx(8:14,k)-[0;0;Param.dZ;0;0;0;0])) + Param.uu(:,k)'*Param.R*Param.uu(:,k) ...
    + (Param.xx(1:2,k)-[-90;37.33])'*Param.S*(Param.xx(1:2,k)-[-90;37.33]);         %um ponto no final da reta predefinida par o shuttle seguir tambem
       %  + (Param.xx(4:6,k)-[8.5;0;0])'*Param.S*(Param.xx(4:6,k)-[8.5;0;0]);         %velocidade do shuttle para a frente tendo em conta a reta predifinida
end

    Param.distance = sqrt((Param.xx(1,Param.N+1)-Param.xx(8,Param.N+1))^2 + (Param.xx(2,Param.N+1)-Param.xx(9,Param.N+1))^2  ) ;
  Param.dZ =  if_else( Param.distance <= Param.closeness,0 ,Param.height );

 % obj = obj + (Param.xx(1:7,Param.N+1)-(Param.xx(8:14,Param.N+1)-[0;0;Param.dZ+0.7;0;0;0;0]))'*Param.Q*(Param.xx(1:7,Param.N+1)-(Param.xx(8:14,Param.N+1)-[0;0;Param.dZ+0.7;0;0;0;0])) ...
obj = obj + (Param.xx(1:7,Param.N+1)-(Param.xx(8:14,Param.N+1)-[0;0;Param.dZ;0;0;0;0]))'*Param.Q*(Param.xx(1:7,Param.N+1)-(Param.xx(8:14,Param.N+1)-[0;0;Param.dZ;0;0;0;0])) ...
    + (Param.xx(1:2,Param.N+1)-[-90;37.33])'*Param.S*(Param.xx(1:2,Param.N+1)-[-90;37.33]); 
                 %   + (Param.xx(4:6,Param.N+1)-[8.5;0;0])'*Param.S*(Param.xx(4:6,Param.N+1)-[8.5;0;0]);  
MPC.minimize(obj);

%backup objective function
% for k = 1:Param.N
%     obj = obj + (Param.xx(1:7,k)-Param.xx(8:14,k))'*Param.Q*(Param.xx(1:7,k)-Param.xx(8:14,k)) + Param.uu(:,k)'*Param.R*Param.uu(:,k);
% end
% obj = obj + (Param.xx(1:7,Param.N+1)-Param.xx(8:14,Param.N+1))'*Param.Q*(Param.xx(1:7,Param.N+1)-Param.xx(8:14,Param.N+1));
% MPC.minimize(obj);

% Dynamic constraints of the multiple shooting
for k=1:Param.N
    MPC.subject_to(Param.xx(:,k+1) == f_disc(Param.xx(:,k),Param.uu(:,k))); % One step integration function predicts
  %MPC.subject_to( Param.xx(11:13,k)'*Param.xx(4:6,k) >= 0);

  MPC.subject_to(Param.xx(3,k) < Param.xx(10,k) - 0.7 );
  % MPC.subject_to(Param.xx(3,k) < Param.xx(10,k) );
    
    MPC.subject_to(-20 <= Param.xx(4:6,k) <= 20);
    % MPC.subject_to(-15 <= Param.uu(1:3,k) <= 15); %accel
end

% opti.subject_to(u_min <= u <= u_max)
MPC.subject_to(Param.xx(:,1) == Param.xx0)
%MPC.subject_to(-8.5 <= vec(Param.xx(4:6,:)) <= 8.5);
%MPC.subject_to(vec(Param.xx(3,:)) < vec(Param.xx(14,:)));



% Solution Value
% use ipopt (mean dt_MPC = 13 ms, max dt_MPC = ~40 ms)
s_opts = struct;
%s_opts.ipopt.print_level =0;
%s_opts.print_time = 0;
% MPC.solver('ipopt',s_opts);

%s_opts.qpsol = 'osqp';
s_opts.qpsol = 'qrqp';
s_opts.print_header = false;
s_opts.print_iteration = false;
s_opts.print_time = false;
s_opts.qpsol_options.print_iter = false;
s_opts.qpsol_options.print_header = false;
s_opts.qpsol_options.print_info = false;
s_opts.qpsol_options.error_on_fail = false;
MPC.solver('sqpmethod',s_opts);


%gen_opts = struct('main',true,'cpp',true,'with_header',true,'with_mem',true);
%MPC.solver.generate('gen.cpp', gen_opts);
Param.MPC = MPC;

%F = MPC.to_function('F',{Param.xx0},{Param.xx, Param.uu},{'xx0'},{'xx','uu'});
%gen_opts = struct('main',true,'cpp',true,'with_header',true,'with_mem',true);
%F.generate('gen.cpp', gen_opts);
%F.save('f.casadi');

%F = MPC.to_function('F',{Param.xx0},{Param.uu(:,1),Param.xx(:,1)},{'xx0'},{'uu_opt','xx_pred'})

F = MPC.to_function('F',{Param.xx0,Param.xx,Param.uu},{Param.uu,Param.xx},{'xx0','xx','uu'},{'uu_opt','xx_pred'})


%F = MPC.to_function('F',{Param.xx0},{Param.uu(:,:),Param.xx(:,:)},{'xx0'},{'uu_opt','xx_pred'})
gen_opts = struct('main',true,'cpp',true,'with_header',true,'with_mem',true);
F.generate('gen.cpp', gen_opts);
F.save('F.casadi');

    x_MPC = zeros(n_states,Param.N+1);
    u_MPC = zeros(n_controls,Param.N);

