
clear all; clc; close all;


states = readmatrix('state_logs.csv');
%states_com = readmatrix('state_logs_com.csv');








 figure(111);
plot3(states(:,8),states(:,9),states(:,10));

hold on;

plot3(states(:,1),states(:,2),states(:,3), '--');


plot3(states(1,8),states(1,9), states(1,10),'square','Color','#0072BD','MarkerSize',8);
plot3(states(1,1),states(1,2),states(1,3),'square','Color','#D95319','MarkerSize',8);

plot3(states(end,1),states(end,2),states(end,3),'x','Color','#D95319','MarkerSize',8);


plot3(73,-24, -22,'+','Color','#D95319','MarkerSize',20,'LineWidth', 2);
plot3(35.35,-9.75,-25,'o','Color','#D95319','MarkerSize',8);
plot3(-10,-15,-3,'o','Color','#0072BD','MarkerSize',8);



%shuttleStopArea -93 15 -25
%                -70 100 -25
%               [23 85 0];

t = -1:2;
plot3(-93 + 23*t, 15 +85*t, -24 + 0*t,'-','Color',[1, 0, 0, 0.25]);
x=-116:5:-47;
y=-70:5:185;
z=-25:1:0;
[X,Y,Z] = meshgrid(x,y,z);
I = ((85/23)*(X-23+93) + 100 <= Y) & (Z==-24) ;

scatter3(X(I),Y(I),Z(I),'.','filled', 'red','MarkerFaceAlpha',0.25, 'MarkerEdgeAlpha', 0.25,'MarkerEdgeColor', 'flat');


title('Target and Shuttle Positions - Failsafe Test');
  xlabel('North [m]');
    ylabel('East [m]');
    zlabel('-Down [m]');
set(gca, 'Zdir', 'reverse');
set(gca, 'Ydir', 'reverse');
%set(gca, 'Xdir', 'reverse');
legend('Target Trajectory', 'Shuttle Trajectory', 'Target Start Position','Shuttle Takeoff Position','Shuttle Landing Position','P_{inform}','P_{wait}','P_{target\_collection}','','A_{stop}','Interpreter','tex');
grid on;
axis equal;
%axis([-50 500 -50 100 -50 50]);
  
hold off;



     figure(222);
  
     plot( (1:size(states))*0.01,states(:,8:10), '-');
     hold on;
        plot((1:size(states))*0.01, states(:,1:3), '--');
     title('Shuttle and Target Positions - Failsafe Test');
     
   %  legend('x_{target}','y_{target}','z_{target}','x_{shuttle}','y_{shuttle}','z_{shuttle}','Interpreter','tex');  
    
     
     
     
     
%     state_machine = states(:,15);
%     takeoff = find(state_machine == 1);
%     xline(takeoff(1)*0.01,':','TAKEOFF','FontSize', 6,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
%     send_to_wainting_point = find(state_machine == 2);
%     xline(send_to_wainting_point(1)*0.01,':','SEND_TO_WAITING_POINT','FontSize', 6,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
%     hover_waiting = find(state_machine == 3);
%     xline(hover_waiting(1)*0.01,':','HOVER_WAITING_FOR_TARGET','FontSize', 6,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
%     activate_mpc = find(state_machine == 4);
%     xline(activate_mpc(1)*0.01,':','ACTIVATE_MPC_AND_START_MOVING','FontSize', 6,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
%     execute_capture = find(state_machine == 5);
%     xline(execute_capture(1)*0.01,':','EXECUTE_CAPTURE_MANEUVER','FontSize', 6,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
%     target_collection = find(state_machine == 6);
%     xline(target_collection(1)*0.01,':','SEND_TO_TARGET_COLLECTION_ZONE','FontSize', 6,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
%     drop_target = find(state_machine == 7);
%     xline(drop_target(1)*0.01,':','DROP_TARGET','FontSize', 6,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
%     landing = find(state_machine == 8);
%     xline(landing(1)*0.01,':','PERFORM_LANDING','FontSize', 6,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
%    
     legend('x_{target}','y_{target}','z_{target}','x_{shuttle}','y_{shuttle}','z_{shuttle}','Interpreter','tex');  
   
    
    

     
 hold off;


 figure(333);
  
     plot((1:size(states))*0.01, states(:,15));
     hold on;
          state_machine = states(:,15);
    takeoff = find(state_machine == 1);
    xline(takeoff(1)*0.01,':','TAKEOFF','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
    send_to_wainting_point = find(state_machine == 2);
    xline(send_to_wainting_point(1)*0.01,':','SEND_TO_WAITING_POINT','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');  
    xline(5339*0.01,':','SEND_TO_WAITING_POINT','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
    hover_waiting = find(state_machine == 3);
    xline(hover_waiting(1)*0.01,':','HOVER_WAITING_FOR_TARGET','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
    xline(7214*0.01,':','HOVER_WAITING_FOR_TARGET','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');   
    activate_mpc = find(state_machine == 4);
    xline(activate_mpc(1)*0.01,':','ACTIVATE_MPC_AND_START_MOVING','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
    xline(8766*0.01,':','ACTIVATE_MPC_AND_START_MOVING','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
    execute_capture = find(state_machine == 5);
    xline(execute_capture(1)*0.01,':','EXECUTE_CAPTURE_MANEUVER','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
    target_collection = find(state_machine == 6);
    xline(target_collection(1)*0.01,':','SEND_TO_TARGET_COLLECTION_ZONE','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
    drop_target = find(state_machine == 7);
    xline(drop_target(1)*0.01,':','DROP_TARGET','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
    landing = find(state_machine == 8);
    xline(landing(1)*0.01,':','PERFORM_LANDING','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'aligned','Interpreter','none');
   
     title('State Machine Current State - Failsafe Test');

       ylabel('Current State');
    xlabel('Time [s]');
    % legend('x_{target}','y_{target}','z_{target}','x_{shuttle}','y_{shuttle}','z_{shuttle}','Interpreter','tex');  
    
 hold off;
 
 
 
 



     figure(21);
    
     plot((1:size(states))*0.01, states(:,8), '-','Color','#0072BD');
    
     hold on;
     title('Shuttle and Target Positions Evolution in North Axis - Failsafe Test');
       plot((1:size(states))*0.01, states(:,1), '--','Color','#D95319','LineWidth',1.1);
      
       
                     state_machine = states(:,15);
    takeoff = find(state_machine == 1);
    xline(takeoff(1)*0.01,':','1','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    send_to_wainting_point = find(state_machine == 2);
    xline(send_to_wainting_point(1)*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');  
    xline(5339*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    hover_waiting = find(state_machine == 3);
    xline(hover_waiting(1)*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(7214*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');   
    activate_mpc = find(state_machine == 4);
    xline(activate_mpc(1)*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(8766*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    execute_capture = find(state_machine == 5);
    xline(execute_capture(1)*0.01,':','5','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    target_collection = find(state_machine == 6);
    xline(target_collection(1)*0.01,':','6','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    drop_target = find(state_machine == 7);
    xline(drop_target(1)*0.01,':','7','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    landing = find(state_machine == 8);
    xline(landing(1)*0.01,':','8','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
   
     legend('p_{n_{target}}','p_{n_{shuttle}}','Interpreter','tex');  
          
  
       ylabel('Position North Axis [m]');
    xlabel('Time [s]');
    
 hold off;
  figure(22);
    
     plot((1:size(states))*0.01, states(:,9), '-','Color','#0072BD');
    
     hold on;
     title('Shuttle and Target Positions Evolution in East Axis - Failsafe Test');
       plot((1:size(states))*0.01, states(:,2), '--','Color','#D95319','LineWidth',1.1);
                                state_machine = states(:,15);
    takeoff = find(state_machine == 1);
    xline(takeoff(1)*0.01,':','1','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    send_to_wainting_point = find(state_machine == 2);
    xline(send_to_wainting_point(1)*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');  
    xline(5339*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    hover_waiting = find(state_machine == 3);
    xline(hover_waiting(1)*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(7214*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');   
    activate_mpc = find(state_machine == 4);
    xline(activate_mpc(1)*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(8766*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    execute_capture = find(state_machine == 5);
    xline(execute_capture(1)*0.01,':','5','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    target_collection = find(state_machine == 6);
    xline(target_collection(1)*0.01,':','6','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    drop_target = find(state_machine == 7);
    xline(drop_target(1)*0.01,':','7','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    landing = find(state_machine == 8);
    xline(landing(1)*0.01,':','8','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
   
     legend('p_{e_{target}}','p_{e_{shuttle}}','Interpreter','tex');  
     
       ylabel('Position East Axis [m]');
    xlabel('Time [s]');
    
 hold off;
 
  figure(23);
    
     plot((1:size(states))*0.01, states(:,10), '-','Color','#0072BD');
    
     hold on;
     title('Shuttle and Target Positions Evolution in -Down Axis - Failsafe Test');
       plot((1:size(states))*0.01, states(:,3), '--','Color','#D95319','LineWidth',1.1);
                                 state_machine = states(:,15);
    takeoff = find(state_machine == 1);
    xline(takeoff(1)*0.01,':','1','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    send_to_wainting_point = find(state_machine == 2);
    xline(send_to_wainting_point(1)*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');  
    xline(5339*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    hover_waiting = find(state_machine == 3);
    xline(hover_waiting(1)*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(7214*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');   
    activate_mpc = find(state_machine == 4);
    xline(activate_mpc(1)*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(8766*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    execute_capture = find(state_machine == 5);
    xline(execute_capture(1)*0.01,':','5','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    target_collection = find(state_machine == 6);
    xline(target_collection(1)*0.01,':','6','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    drop_target = find(state_machine == 7);
    xline(drop_target(1)*0.01,':','7','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    landing = find(state_machine == 8);
    xline(landing(1)*0.01,':','8','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
   
     legend('p_{d_{target}}','p_{d_{shuttle}}','Interpreter','tex');  
     set(gca, 'Ydir', 'reverse');
       ylabel('Position -Down Axis [m]');
    xlabel('Time [s]');

 hold off;
 
 
 
 
 
 
 hold off;
     figure(3);
    
      plot((1:size(states))*0.01, states(:,14) , '-','Color','#0072BD');
     hold on;
     title('Shuttle and Target  \psi  Evolution - Failsafe Test','Interpreter','tex');
     plot((1:size(states))*0.01, states(:,7), '--','Color','#D95319','LineWidth',1.1);
    
                              state_machine = states(:,15);
    takeoff = find(state_machine == 1);
    xline(takeoff(1)*0.01,':','1','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    send_to_wainting_point = find(state_machine == 2);
    xline(send_to_wainting_point(1)*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');  
    xline(5339*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    hover_waiting = find(state_machine == 3);
    xline(hover_waiting(1)*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(7214*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');   
    activate_mpc = find(state_machine == 4);
    xline(activate_mpc(1)*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(8766*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    execute_capture = find(state_machine == 5);
    xline(execute_capture(1)*0.01,':','5','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    target_collection = find(state_machine == 6);
    xline(target_collection(1)*0.01,':','6','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    drop_target = find(state_machine == 7);
    xline(drop_target(1)*0.01,':','7','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    landing = find(state_machine == 8);
    xline(landing(1)*0.01,':','8','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
   
     legend('\psi_{target}','\psi_{shuttle}','Interpreter','tex');  
      ylabel('\psi [rad]');
    xlabel('Time [s]');

 hold off;
 


hold off;

  figure(41);
    
     plot((1:size(states))*0.01, states(:,11), '-','Color','#0072BD');
    
     hold on;
     title('Shuttle and Target Velocity Evolution in North Axis - Failsafe Test');
       plot((1:size(states))*0.01, states(:,4), '--','Color','#D95319','LineWidth',1.1);
     
                              state_machine = states(:,15);
    takeoff = find(state_machine == 1);
    xline(takeoff(1)*0.01,':','1','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    send_to_wainting_point = find(state_machine == 2);
    xline(send_to_wainting_point(1)*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');  
    xline(5339*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    hover_waiting = find(state_machine == 3);
    xline(hover_waiting(1)*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(7214*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');   
    activate_mpc = find(state_machine == 4);
    xline(activate_mpc(1)*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(8766*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    execute_capture = find(state_machine == 5);
    xline(execute_capture(1)*0.01,':','5','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    target_collection = find(state_machine == 6);
    xline(target_collection(1)*0.01,':','6','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    drop_target = find(state_machine == 7);
    xline(drop_target(1)*0.01,':','7','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    landing = find(state_machine == 8);
    xline(landing(1)*0.01,':','8','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    
     legend('v_{n_{target}}','v_{n_{shuttle}}','Interpreter','tex');  
     
       ylabel('Velocity North Axis [m/s]');
    xlabel('Time [s]');
    
 hold off;

 figure(42);
    
     plot((1:size(states))*0.01, states(:,12), '-','Color','#0072BD');
    
     hold on;
     title('Shuttle and Target Velocity Evolution in East Axis - Failsafe Test');
       plot((1:size(states))*0.01, states(:,5), '--','Color','#D95319','LineWidth',1.1);
     
                          state_machine = states(:,15);
    takeoff = find(state_machine == 1);
    xline(takeoff(1)*0.01,':','1','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    send_to_wainting_point = find(state_machine == 2);
    xline(send_to_wainting_point(1)*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');  
    xline(5339*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    hover_waiting = find(state_machine == 3);
    xline(hover_waiting(1)*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(7214*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');   
    activate_mpc = find(state_machine == 4);
    xline(activate_mpc(1)*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(8766*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    execute_capture = find(state_machine == 5);
    xline(execute_capture(1)*0.01,':','5','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    target_collection = find(state_machine == 6);
    xline(target_collection(1)*0.01,':','6','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    drop_target = find(state_machine == 7);
    xline(drop_target(1)*0.01,':','7','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    landing = find(state_machine == 8);
    xline(landing(1)*0.01,':','8','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
      legend('v_{e_{target}}','v_{e_{shuttle}}','Interpreter','tex');  
     
       ylabel('Velocity East Axis [m/s]');
    xlabel('Time [s]');
    
 hold off;
 figure(43);
    
     plot((1:size(states))*0.01, states(:,13), '-','Color','#0072BD');
    
     hold on;
     title('Shuttle and Target Velocity Evolution in -Down Axis - Failsafe Test');
       plot((1:size(states))*0.01, states(:,6), '--','Color','#D95319','LineWidth',1.1);
      
                        state_machine = states(:,15);
    takeoff = find(state_machine == 1);
    xline(takeoff(1)*0.01,':','1','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    send_to_wainting_point = find(state_machine == 2);
    xline(send_to_wainting_point(1)*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');  
    xline(5339*0.01,':','2','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    hover_waiting = find(state_machine == 3);
    xline(hover_waiting(1)*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(7214*0.01,':','3','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');   
    activate_mpc = find(state_machine == 4);
    xline(activate_mpc(1)*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    xline(8766*0.01,':','4','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    execute_capture = find(state_machine == 5);
    xline(execute_capture(1)*0.01,':','5','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    target_collection = find(state_machine == 6);
    xline(target_collection(1)*0.01,':','6','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    drop_target = find(state_machine == 7);
    xline(drop_target(1)*0.01,':','7','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    landing = find(state_machine == 8);
    xline(landing(1)*0.01,':','8','FontSize', 8,'Alpha',0.4, 'LabelOrientation', 'horizontal','Interpreter','none');
    
     legend('v_{d_{target}}','v_{d_{shuttle}}','Interpreter','tex');  
     
       ylabel('Velocity -Down Axis [m/s]');
        set(gca, 'Ydir', 'reverse');
    xlabel('Time [s]');
    
 hold off;




  figure(6);
  
    mpc_time = states(:,16);
    timeNo0 = mpc_time(mpc_time ~=0);
    active = find( timeNo0 ~= 1111);
    
     plot( (active(1):active(end))*0.01 , timeNo0(active(1):active(end)));
  
     hold on;
     title('MPC Computation Time - Failsafe Test');
       
       ylabel('MPC Computations Duration [s]');
    xlabel('Time [s]');
     %plot(1:sim_time/T + 1, target_states(1:3,:), '--');
     %legend('x_{shuttle}','y_{shuttle}','z_{shuttle}','x_{target}','y_{target}','z_{target}','Interpreter','tex');  
    
 hold off;
 
 
 
 hold off;



     figure(555);
  mpc_active = find(  states(:,15) == 4 | states(:,15) == 5);
     plot((mpc_active(1):mpc_active(end))*0.01, states(mpc_active(1):mpc_active(end),8:10), '-');
     hold on;
        plot( (mpc_active(1):mpc_active(end))*0.01,states(mpc_active(1):mpc_active(end),1:3), '--');
     title('Shuttle and Target Positions While the MPC is Active - Failsafe Test');
   ylabel('Position [m]');
    xlabel('Time [s]');
     legend('x_{target}','y_{target}','z_{target}','x_{shuttle}','y_{shuttle}','z_{shuttle}','Interpreter','tex');  
    
 hold off;

 
     figure(666);
  mpc_active = find(  states(:,15) == 4 | states(:,15) == 5);
  distance =  abs( states(mpc_active(1):mpc_active(end),10) - states(mpc_active(1):mpc_active(end),3));
  
    plot((mpc_active(1):mpc_active(end))*0.01, abs( states(mpc_active(1):mpc_active(end),10) - states(mpc_active(1):mpc_active(end),3)), '-');
     hold on;
     
    % [min_distance,index] =min(distance);
      z_contact_point = min(distance)
     title('Distance Between Drones in the Down Axis While the MPC is Active - Failsafe Test');
  ylabel('Distance Down Axis [m]');

    xlabel('Time [s]');
     %legend('x_{target}','y_{target}','z_{target}','x_{shuttle}','y_{shuttle}','z_{shuttle}','Interpreter','tex');  
    
 hold off;

  
     figure(777);
  mpc_active = find(  states(:,15) == 4 | states(:,15) == 5);
  distance =  sqrt(( states(mpc_active(1):mpc_active(end),9) - states(mpc_active(1):mpc_active(end),2)).^2 + ( states(mpc_active(1):mpc_active(end),8) - states(mpc_active(1):mpc_active(end),1)).^2);
  
    plot((mpc_active(1):mpc_active(end))*0.01, sqrt(( states(mpc_active(1):mpc_active(end),9) - states(mpc_active(1):mpc_active(end),2)).^2 + ( states(mpc_active(1):mpc_active(end),8) - states(mpc_active(1):mpc_active(end),1)).^2), '-');
     hold on;
     
     
      xy_contact_point = min(distance)
    
      title('Distance Between Drones in the North and East Axis While the MPC is Active - Failsafe Test');
  ylabel('Distance North and East Axis [m]');

    xlabel('Time [s]');
     %legend('x_{target}','y_{target}','z_{target}','x_{shuttle}','y_{shuttle}','z_{shuttle}','Interpreter','tex');  
    
 hold off;
 





%      figure(6666);
%   mpc_active = find(  states(:,15) == 4 | states(:,15) == 5);
%   distance =  abs( states(mpc_active(1):mpc_active(end),10) - states(mpc_active(1):mpc_active(end),3));
%   
%     plot((mpc_active(1):mpc_active(end))*0.01, abs( states(mpc_active(1):mpc_active(end),10) - states(mpc_active(1):mpc_active(end),3)), '-');
%      hold on;
% 
%      
%       mpc_active = find(  states_com(:,15) == 4 | states_com(:,15) == 5);
%   distance =  abs( states_com(mpc_active(1):mpc_active(end),10) - states_com(mpc_active(1):mpc_active(end),3));
%   
%     plot((mpc_active(1):mpc_active(end))*0.01, abs( states_com(mpc_active(1):mpc_active(end),10) - states_com(mpc_active(1):mpc_active(end),3)), '-');
%      hold on;
% 
%      title('Distance Between Drones in the Down Axis - MPC Comparison');
%   ylabel('Distance Down Axis [m]');
% 
%     xlabel('Time [s]');
%      legend('MPC Without Velocity Constraint','MPC With Velocity Constraint','Interpreter','tex');  
%     
%  hold off;
% 
%   
%      figure(7777);
%   mpc_active = find(  states(:,15) == 4 | states(:,15) == 5);
%   distance =  sqrt(( states(mpc_active(1):mpc_active(end),9) - states(mpc_active(1):mpc_active(end),2)).^2 + ( states(mpc_active(1):mpc_active(end),8) - states(mpc_active(1):mpc_active(end),1)).^2);
%   
%     plot((mpc_active(1):mpc_active(end))*0.01, sqrt(( states(mpc_active(1):mpc_active(end),9) - states(mpc_active(1):mpc_active(end),2)).^2 + ( states(mpc_active(1):mpc_active(end),8) - states(mpc_active(1):mpc_active(end),1)).^2), '-');
%      hold on;
%      
%      
%      mpc_active = find(  states_com(:,15) == 4 | states_com(:,15) == 5);
%   distance =  sqrt(( states_com(mpc_active(1):mpc_active(end),9) - states_com(mpc_active(1):mpc_active(end),2)).^2 + ( states_com(mpc_active(1):mpc_active(end),8) - states_com(mpc_active(1):mpc_active(end),1)).^2);
%   
%     plot((mpc_active(1):mpc_active(end))*0.01, sqrt(( states_com(mpc_active(1):mpc_active(end),9) - states_com(mpc_active(1):mpc_active(end),2)).^2 + ( states_com(mpc_active(1):mpc_active(end),8) - states_com(mpc_active(1):mpc_active(end),1)).^2), '-');
%      hold on;
%      
%     
%       title('Distance Between Drones in the North and East Axis - MPC Comparison');
%   ylabel('Distance North and East Axis [m]');
% 
%     xlabel('Time [s]');
%      legend('MPC Without Velocity Constraint','MPC With Velocity Constraint','Interpreter','tex');  
%     
%  hold off;
%  



 if 0
 
      figure(112221);
    curve = animatedline('LineWidth',2,'Color','#D95319');
    curve2 = animatedline('LineWidth',1,'Color','#0072BD');
    %set(gca,'XLim',[-1.5 1.5],'YLim',[-1.5 1.5],'ZLim',[0 10]);
    view(43,24);
     set(gca, 'Zdir', 'reverse');
     set(gca, 'Ydir', 'reverse');
      grid on;
     axis equal;
     %axis([-60 610 -50 230 -20 0]); %full view
     axis([-40 50 -20 20 -27 -15]); %capture maneuver
     title('Target and Shuttle Positions');
     xlabel('x[m]'); ylabel('y[m]'); zlabel('-z[m]');
    % legend('Target', 'Shuttle');
    hold on;
    for i = 1:size(states(:,8),1)
       addpoints(curve2,states(i,8),states(i,9),states(i,10));
        head2 = scatter3(states(i,8),states(i,9),states(i,10),'filled','MarkerFaceColor','b');
         addpoints(curve,states(i,1),states(i,2),states(i,3));
        head = scatter3(states(i,1),states(i,2),states(i,3),'filled','MarkerFaceColor','b');

        drawnow
        pause(0.001);
        delete(head);
        delete(head2);
    end
 end

hold off;
