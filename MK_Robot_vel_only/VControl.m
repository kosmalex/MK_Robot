classdef VControl
  %VCONTROL a class wrapper for the velocity controller
  %of the MK-Robot
  
  properties
    Kappa = randi(2,2);
    lr = 0; %From Learning Rate
    
    U = @(obj, d_vec_col) obj.Kappa * d_vec_col;
  end
  
  methods
    function obj = VControl(Kappa, learning_rate)
      obj.lr = learning_rate;
      obj.Kappa = Kappa;
    end
    
    function area_of_norm = SMC(obj, mk_robot, step, MAX_ITERATIONS, should_plot_trajectory)
      %METHOD1 basically computes the area of sum(x.^2)
      
      robot_trajectory = zeros(MAX_ITERATIONS, 2);
      robot_velocity = zeros(MAX_ITERATIONS, 2);
      
      area_of_norm = 0; %This is an indicator of performance
      iterator = 1;
      while(iterator <= MAX_ITERATIONS)
        if should_plot_trajectory == 1
          robot_trajectory(iterator, :) = mk_robot.pos';
          robot_velocity(iterator, :) = mk_robot.v';
        end
        
        error_vec = mk_robot.get_error();
        velocity_vec = obj.U(obj, error_vec);
        mk_robot = mk_robot.move(velocity_vec, step);
        
        area_of_norm = area_of_norm + sum(abs(mk_robot.get_error()).^2);
        
        iterator = iterator + 1;
      end
      
      if should_plot_trajectory == 1
        sampling = 1:MAX_ITERATIONS;
        quiver(robot_trajectory(sampling, 1), robot_trajectory(sampling, 2),robot_velocity(sampling, 1),robot_velocity(sampling, 2), 'r', 'LineWidth', 2, 'AutoScale', 'on', 'AutoScaleFactor', 0.4);
      end
    end
  end
end

