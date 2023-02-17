function joint_ID = ID_planarRR(q_des,q_dot_des,q_ddot_des,robot_params,navigation_params,euler_params)

    Kv = 10; %gain
    q = navigation_params.x_ini'; q_dot = [0;0];
    joint_ID.angles = q';
    joint_ID.vel = q_dot';
    joint_ID.control = [0;0]';
    dt = euler_params.step_size; N = euler_params.n_steps;
    
    % wrap around SE2 torous for q1 and q2
    if(q(1) > 2*pi)
        disp('wrap:0');
        q(1) = q(1,1) - 2*pi; 
    end
    if(q(2,1) > 2*pi)
        disp('wrap:0')
        q(2) = q(2,1) - 2*pi; 
    end
    if(q(1) < 0)
        disp('wrap:0')
        q(1) = 2*pi + q(1,1);
    end
    if(q(1) < 0)
        disp('wrap:0')
        q(2) = 2*pi + q(2,1);
    end

    for i = 1:N-1
        x = [q;q_dot];
        %define errors
        e       = q - q_des(i,:)'; 
        e_dot   = q_dot - q_dot_des(:,i);
        
        % get M C G matrices
        t= 0; u = [0;0];
        dynamics = dynamics_planarRR(t, x, u, robot_params);
        M = dynamics.M; C = dynamics.C; G = dynamics.G;

        % inverse dynamics control
        u_id = M*(q_ddot_des(:,i)) + C + G + M*(grad_density_f(e)- Kv.*e_dot);
        

        % apply control to simulate using euler
        dynamics = dynamics_planarRR(t, x, u_id, robot_params);
        x_update = x + dt.*dynamics.f;
        
        % update states
        q = x_update(1:2); q_dot = x_update(3:4);

         % wrap around SE2 torous for q1 and q2
        if(q(1) > 2*pi)
            disp('wrap:'); disp(n);
            q(1) = q(1,1) - 2*pi; 
        end
        if(q(2) > 2*pi)
            disp('wrap:'); disp(n);
            q(2) = q(2,1) - 2*pi;
        end
        if(q(1) < 0)
            disp('wrap:'); disp(n);
            q(1) = 2*pi + q(1,1);
        end
         if(q(1) < 0)
            disp('wrap:'); disp(n);
            q(2) = 2*pi + q(2,1);
        end 

        joint_ID.angles = [joint_ID.angles; q'];
        joint_ID.vel = [joint_ID.vel; q_dot'];
        joint_ID.control = [joint_ID.control; u_id'];
    end

end
