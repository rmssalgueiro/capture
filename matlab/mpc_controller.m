function [control_action,x_seq_next,u_seq_next] = mpc_controller(current_state,P,x_seq_prev,u_seq_prev, mpc_external_function, aux1)

        use_external_function = 1;
        
        if ~use_external_function

            P.MPC.set_value(P.xx0,current_state);
            P.MPC.set_initial(P.xx,x_seq_prev);
            P.MPC.set_initial(P.uu,u_seq_prev);
            sol = P.MPC.solve();
            x_seq = sol.value(P.xx);
            u_seq = sol.value(P.uu);
            x_seq_next = [x_seq(:,2:end),x_seq(:,end)];
            u_seq_next = [u_seq(:,2:end),u_seq(:,end)];
            %control_action = P.m*u_seq(1:3,1) + P.m*P.g*zW;
            control_action = [P.m*u_seq(1:3,1) - P.m*P.g*[0;0;1];u_seq(4,1)];
            %control_action = u_seq;
        
        else
            if aux1 == 1
                current_state(10) = current_state(10) +  -2;
            end
                
            [u_seq, x_seq] = mpc_external_function(current_state,u_seq_prev,x_seq_prev);
            control_action = [P.m*u_seq(1:3,1) - P.m*P.g*[0;0;1];u_seq(4,1)];
        
            
            x_seq_next = x_seq;
            u_seq_next = u_seq;
            
        end
end

