classdef VControl
  %VCONTROL Summary of this class goes here
  %   Detailed explanation goes here
  
  properties
    Kappa = randi(2,2);
    Lamda = randi(2,2);
    Beta  = randi(2,1);
    lr = 0; %From Learning Rate
    
    U_p2v = @(obj, d_vec_col) obj.Kappa * d_vec_col;
    U_v2u = @(obj, dvel, curvel) obj.Beta' * curvel + obj.Lamda * (dvel-curvel);
  end
  
  methods
    function obj = VControl(Robot, Kappa, Lamda, learning_rate)
      const_expr = (4*(Robot.d^2)*Robot.k_1) / Robot.R^2;
      obj.Beta = [-const_expr; -const_expr];

      obj.Lamda = Lamda;

      obj.lr = learning_rate;
      obj.Kappa = Kappa;
    end
    
    function [area_of_norm_p, area_of_norm_v] = SMC(obj, mk_robot, step, MAX_ITERATIONS, should_plot_trajectory)
      %SMC basically computes the area of sum(x.^2)
      
      robot_trajectory = zeros(MAX_ITERATIONS, 2);
      robot_velocity = zeros(MAX_ITERATIONS, 2);
      
      area_of_norm_p = 0; %This is an indicator of performance
      area_of_norm_v = 0; %This is an indicator of performance
      iterator = 1;
      while(iterator <= MAX_ITERATIONS)
        if should_plot_trajectory == 1
          robot_trajectory(iterator, :) = mk_robot.pos';
          robot_velocity(iterator, :) = mk_robot.v';
        end
        
        %--------------- First controller ------------------%
        error_vec = mk_robot.get_error();
        desired_vel = obj.U_p2v(obj, error_vec);

        %--------------- Second controller ------------------%
        input_torque = obj.U_v2u(obj, desired_vel, mk_robot.v);
        N = mk_robot.calc_N();
        res_vel = mk_robot.v + step*(mk_robot.M^-1)*( N + mk_robot.B * input_torque);  
        
        %---------------- Robot update --------------%
        mk_robot = mk_robot.move(res_vel, step);
        
        area_of_norm_p = area_of_norm_p + sum(abs(mk_robot.get_error()).^2);
        area_of_norm_v = area_of_norm_v + sum(abs(desired_vel-mk_robot.v).^2);
        
        iterator = iterator + 1;
      end
      
      if should_plot_trajectory == 1
        sampling = 1:MAX_ITERATIONS;
        quiver(robot_trajectory(sampling, 1), robot_trajectory(sampling, 2),robot_velocity(sampling, 1),robot_velocity(sampling, 2), 'r', 'LineWidth', 2, 'AutoScale', 'on', 'AutoScaleFactor', 0.4);
      end
    end
  end
end

