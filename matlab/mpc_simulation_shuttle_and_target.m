
clear all; clc; close all;


addpath('D:\Docments\Matlab\toolbox\casadi-3.6.3-windows64-matlab2018b')
import casadi.*


set_param_fixed_complex;


mpc_setup;
mpc_external_function = Function.load('F.casadi');

Ts =Param.T/Param.N;
Param.m = 4;
Param.g = 9.81;

%% Defining Target System ------------------------------------------------------------------------------------------

% Constrains of UAV

%input constrains
v_target_min = 10; v_target_max = 20;
psi_target_min = -inf; psi_target_max = inf;
  
%states constrains

gamma_target_min = -pi/6; gamma_target_max = pi/6;
phi_target_min = -pi/6; phi_target_max = pi/6;

% States of UAV 

x_target = SX.sym('x_target'); y_target = SX.sym('y_target'); z_target = SX.sym('z_target'); psi_target = SX.sym('psi_target');   %states of the UAV
states_target = [x_target; y_target; z_target; psi_target];      n_states_target = length(states_target);               %UAV

% Controls of UAV 

v_target = SX.sym('v_target'); gamma_target = SX.sym('gamma_target'); phi_target = SX.sym('phi_target');         % UAV cotrols parameters

controls_target = [v_target; gamma_target; phi_target]; n_controls_target = length(controls_target);
rhs_target = [v_target*cos(psi_target)*cos(gamma_target); v_target*sin(psi_target)*cos(gamma_target); -v_target*sin(gamma_target); (9.81/v_target)*tan(phi_target)];                                                        %systems r.h.s

%possivelmente nao vai ser usado e depois é na funcao shitf que se faz o controlo do target
f_target = Function('f_target',{states_target,controls_target}, {rhs_target});                                         % Nonlinear Mapping Function f(x,u)


%% Defining Shuttle System ------------------------------------------------------------------------------------------

% Constrains of UAV

%input constrains
%v_target_min = 10; v_target_max = 20;
%psi_target_min = -inf; psi_target_max = inf;
  
%states constrains

%gamma_target_min = -pi/6; gamma_target_max = pi/6;
%phi_target_min = -pi/6; phi_target_max = pi/6;

% States of UAV 




p_shuttle = MX.sym('p_shuttle',3); v_shuttle = MX.sym('v_shuttle',3); psi_shuttle = MX.sym('psi_shuttle',1);
states_shuttle = [p_shuttle; v_shuttle; psi_shuttle];      n_states_shuttle = size(states_shuttle,1);               %UAV

% Controls of UAV 
a_shuttle = MX.sym('a_shuttle',3); omegaz_shuttle = MX.sym('omegaz_shuttle',1);
controls_shuttle = [a_shuttle;omegaz_shuttle];                      n_controls_shuttle = size(controls_shuttle,1);


rhs_shuttle = [v_shuttle;1/Param.m*a_shuttle+[0;0;1]*Param.g;omegaz_shuttle];                                                        %systems r.h.s


f_shuttle = Function('f_shuttle',{states_shuttle,controls_shuttle}, {rhs_shuttle});                                         % Nonlinear Mapping Function f(x,u)







%% Simulation ---------------------------------------------------------------------------------------------
t0 = 0;
xt = [ParamFixComplex.p0;ParamFixComplex.psi0]; %initial location of target drone

p_shuttle =[200; 10; -15];
v_shuttle =[0;0;0];
psi_shuttle = 0;


current_path_segments = [];
mpc_time_delay = [];

target_reached_inform_point = 0;
inform_point = [160;20;-10];


t(1) = t0;
sim_time = 160;

% NMPC starts form here

k = 1; %k = mpciter
                                
target_states = [];
target_states(:,1) = xt;

shuttle_states = []; %states in mpc, with shuttle and target dynamics
shuttle_states(:,1) = [p_shuttle;v_shuttle;psi_shuttle;[xt(1);xt(2);xt(3)]; [0;0;0]; xt(4)];
%shuttle_states(:,1) = [p_shuttle(1),p_shuttle(2),p_shuttle(3),v_shuttle(1),v_shuttle(2),v_shuttle(3),psi_shuttle,xt(1),xt(2),xt(3), 0;0;0, xt(4)];

aux = 0 ;
aux1= 0;
main_loop = tic;
while (k < sim_time/Ts +1)
    
    
    
    
%     if aux > 25
%         aux1 = 1;
%     end
    if target_reached_inform_point
        mpc_tic = tic;
        [u_shuttle,x_MPC,u_MPC] = mpc_controller(shuttle_states(:,k),Param,x_MPC,u_MPC, mpc_external_function,aux1);
        %u_shuttle = 0;
        mpc_toc = toc(mpc_tic);
        mpc_time_delay= horzcat(mpc_time_delay, mpc_toc);
        
    else
        u_shuttle = [0;0; - Param.m*Param.g;0];
    end
    t(k) = t0;
    
    [path_segment, path_type]  = complex_path_manager(target_states(1:3,k),  ParamFixComplex, current_path_segments);
    
    
  
    [t0, xt, xs] = apply_controls(Ts, t0, f_target, xt,f_shuttle,shuttle_states(:,k),u_shuttle(:,1), ParamFixComplex, path_segment, path_type ); % get the initialization of the next optimization step
                                                                          %apply control action to the system
                                                                        %discretizacao/avancar o tempo
    target_states(:,k+1) = xt;
    shuttle_states(:,k+1) = [xs;xt(1:3,:);(xt(1:3,:)-shuttle_states(8:10,k))/Ts;xt(4,:)];
    
    

    
    if  (sqrt((inform_point(1) - xt(1))^2 + (inform_point(2) - xt(2))^2)) < 5 
        target_reached_inform_point = 1;
    end
    
    if abs(xt(3)-xs(3)) < 0.1
        aux = aux +1;        
    end 
    %k,t0
    k = k + 1;

end
main_loop_time = toc(main_loop)

mpc_average_computation_time = mean(mpc_time_delay);
mpc_average_computation_time
%% Plots----------------------------------------------------------------------------------------------------------------------
 figure(1);
plot3(target_states(1,:),target_states(2,:), target_states(3,:),'-','Color','#0072BD');
hold on;
plot3(shuttle_states(1,:),shuttle_states(2,:),shuttle_states(3,:),'--','Color','#D95319','LineWidth',1.5);
 
plot3(target_states(1,1),target_states(2,1), target_states(3,1),'o','Color','#0072BD','MarkerSize',8);
plot3(shuttle_states(1,1),shuttle_states(2,1),shuttle_states(3,1),'o','Color','#D95319','MarkerSize',8);

title('Target and Shuttle Trajectories');

  xlabel('North [m]');
    ylabel('East [m]');
    zlabel('-Down [m]');
set(gca, 'Zdir', 'reverse');
legend('Target Drone Trajectory', 'Shuttle Drone Trajectory', 'Target Drone Start Position','Shuttle Drone Start Position');
grid on;
axis equal;
axis([-50 650 -50 250 -30 30]);
  
hold off;


figure(2222);
    
     plot(Ts*(1:sim_time/Ts + 1), sqrt((target_states(1,:) - shuttle_states(1,:)).^2 + (target_states(2,:) - shuttle_states(2,:)).^2), '-','Color','#0072BD');
    
     hold on;
     title('North and East Position Error Evolution During Capture Maneuver');
       %plot(Ts*(1:sim_time/Ts + 1), shuttle_states(1,:), '--','Color','#D95319','LineWidth',1.1);
     
    
       ylabel('Error [m]');
    xlabel('Time [s]');
     axis([115 140 0 7 ]);
 hold off;
figure(3333);
    
     plot(Ts*(1:sim_time/Ts + 1), sqrt((target_states(3,:) - shuttle_states(3,:)).^2 ), '-','Color','#0072BD');
    
     hold on;
     title('Down Position Error Evolution During Capture Maneuver');
       %plot(Ts*(1:sim_time/Ts + 1), shuttle_states(1,:), '--','Color','#D95319','LineWidth',1.1);
     
     legend('p_{n_{target}}','p_{n_{shuttle}}','Interpreter','tex');  
     
        ylabel('Error [m]');
    xlabel('Time [s]');
     axis([115 140 0 7 ]);
 hold off;
     figure(21);
    
     plot(Ts*(1:sim_time/Ts + 1), target_states(1,:), '-','Color','#0072BD');
    
     hold on;
     title('Shuttle and Target Positions Evolution in North Axis');
       plot(Ts*(1:sim_time/Ts + 1), shuttle_states(1,:), '--','Color','#D95319','LineWidth',1.1);
     
     legend('p_{n_{target}}','p_{n_{shuttle}}','Interpreter','tex');  
     
       ylabel('Position North Axis [m]');
    xlabel('Time [s]');
    
 hold off;
  figure(22);
    
     plot(Ts*(1:sim_time/Ts + 1), target_states(2,:), '-','Color','#0072BD');
    
     hold on;
     title('Shuttle and Target Positions Evolution in East Axis');
       plot(Ts*(1:sim_time/Ts + 1), shuttle_states(2,:), '--','Color','#D95319','LineWidth',1.1);
     
     legend('p_{e_{target}}','p_{e_{shuttle}}','Interpreter','tex');  
     
       ylabel('Position East Axis [m]');
    xlabel('Time [s]');
    
 hold off;
 
  figure(23);
    
     plot(Ts*(1:sim_time/Ts + 1), target_states(3,:), '-','Color','#0072BD');
    
     hold on;
     title('Shuttle and Target Positions Evolution in -Down Axis');
       plot(Ts*(1:sim_time/Ts + 1), shuttle_states(3,:), '--','Color','#D95319','LineWidth',1.1);
     
     legend('p_{d_{target}}','p_{d_{shuttle}}','Interpreter','tex');  
     set(gca, 'Ydir', 'reverse');
       ylabel('Position -Down Axis [m]');
    xlabel('Time [s]');
    axis([0 180 -20 0 ]);
 hold off;
 
 
 
 
 
 
 hold off;
     figure(3);
   
     for i = 2:size(target_states(4,:),2)
     
        if target_states(4,i) > pi
            target_states(4,i) = target_states(4,i) - 2*pi;
        end
        

     end
       
        for i = 2:size(shuttle_states(4,:),2)
     
        if shuttle_states(7,i) > pi
            shuttle_states(7,i) = shuttle_states(7,i) - 2*pi;
        end
        

     end
       
  
       
      plot(Ts*(1:sim_time/Ts + 1), target_states(4,:), '-','Color','#0072BD');
     hold on;
     title('Shuttle and Target  \psi  Evolution','Interpreter','tex');
     plot(Ts*(1:sim_time/Ts + 1), shuttle_states(7,:), '--','Color','#D95319','LineWidth',1.1);
     legend('\psi_{target}','\psi_{shuttle}','Interpreter','tex');  
      ylabel('\psi [rad]');
    xlabel('Time [s]');

 hold off;
 


hold off;

  figure(41);
    
     plot(Ts*(1:sim_time/Ts + 1), shuttle_states(11,:), '-','Color','#0072BD');
    
     hold on;
     title('Shuttle and Target Velocity Evolution in North Axis');
       plot(Ts*(1:sim_time/Ts + 1), shuttle_states(4,:), '--','Color','#D95319','LineWidth',1.1);
     
     legend('v_{n_{target}}','v_{n_{shuttle}}','Interpreter','tex');  
     
       ylabel('Velocity North Axis [m/s]');
    xlabel('Time [s]');
    
 hold off;

 figure(42);
    
     plot(Ts*(1:sim_time/Ts + 1), shuttle_states(12,:), '-','Color','#0072BD');
    
     hold on;
     title('Shuttle and Target Velocity Evolution in East Axis');
       plot(Ts*(1:sim_time/Ts + 1), shuttle_states(5,:), '--','Color','#D95319','LineWidth',1.1);
     
     legend('v_{e_{target}}','v_{e_{shuttle}}','Interpreter','tex');  
     
       ylabel('Velocity East Axis [m/s]');
    xlabel('Time [s]');
    
 hold off;
 figure(43);
    
     plot(Ts*(1:sim_time/Ts + 1), shuttle_states(13,:), '-','Color','#0072BD');
    
     hold on;
     title('Shuttle and Target Velocity Evolution in -Down Axis');
       plot(Ts*(1:sim_time/Ts + 1), shuttle_states(6,:), '--','Color','#D95319','LineWidth',1.1);
     
     legend('v_{d_{target}}','v_{d_{shuttle}}','Interpreter','tex');  
     
       ylabel('Velocity -Down Axis [m/s]');
        set(gca, 'Ydir', 'reverse');
    xlabel('Time [s]');
    
 hold off;



  figure(5);
  
    error = vecnorm(shuttle_states(1:3,:) - target_states(1:3,:));
     plot(Ts*(1:sim_time/Ts + 1), error, '-');
    
     hold on;
     title('ponto de contacto');
      
     %plot(1:sim_time/T + 1, target_states(1:3,:), '--');
     %legend('x_{shuttle}','y_{shuttle}','z_{shuttle}','x_{target}','y_{target}','z_{target}','Interpreter','tex');  
    
 hold off;
  figure(6);
  
    %error = vecnorm(shuttle_states(1:3,:) - target_states(1:3,:));
     plot(Ts*(1:size(mpc_time_delay,2)) , mpc_time_delay);
  %  1:sim_time/Ts
     hold on;
     title('MPC Computation Time');
       
       ylabel('MPC Computations Duration [s]');
    xlabel('Time [s]');
     %plot(1:sim_time/T + 1, target_states(1:3,:), '--');
     %legend('x_{shuttle}','y_{shuttle}','z_{shuttle}','x_{target}','y_{target}','z_{target}','Interpreter','tex');  
    
 hold off;
 
 if 0
 
      figure(111);
    curve = animatedline('LineWidth',2,'Color','#D95319');
    curve2 = animatedline('LineWidth',1,'Color','#0072BD');
    %set(gca,'XLim',[-1.5 1.5],'YLim',[-1.5 1.5],'ZLim',[0 10]);
    view(43,24);
     set(gca, 'Zdir', 'reverse');
      grid on;
     axis equal;
     %axis([-60 610 -50 230 -20 0]); %full view
     axis([190 250 -50 50 -20 0]); %capture maneuver
     title('Target and Shuttle Positions');
     xlabel('x[m]'); ylabel('y[m]'); zlabel('-z[m]');
    % legend('Target', 'Shuttle');
    hold on;
    for i = 1:size(target_states(1,:),2)
       addpoints(curve2,target_states(1,i),target_states(2,i),target_states(3,i));
        head2 = scatter3(target_states(1,i),target_states(2,i),target_states(3,i),'filled','MarkerFaceColor','b');
         addpoints(curve,shuttle_states(1,i),shuttle_states(2,i),shuttle_states(3,i));
        head = scatter3(shuttle_states(1,i),shuttle_states(2,i),shuttle_states(3,i),'filled','MarkerFaceColor','b');

        drawnow
        pause(0.001);
        delete(head);
        delete(head2);
    end
 end

hold off;



