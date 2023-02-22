classdef MK_Robot
  %A class wrapper to emulate an MK-robot's position.
  %   Detailed explanation goes here
  
  properties
%----Constant members----%
    m_c = -1;
    m_o = -1;
    m_k = -1;
    l = -1;
    l_c = -1;
    d = -1;
    r_c = -1;
    R = -1;
    k_1 = -1;
    k_2 = -1;
    
    M = zeros(2);
    B = zeros(2);
    
    target_pos = [inf inf]';
    
%----Variable members----%
    %Per wheel rotation angle
    phi_1 = -1;
    phi_2 = -1;
    phi = [-1, -1]';
    
    %Robot's position
    pos = [inf inf]';
    x = inf;
    y = inf;
    
    %Robot's velocity
    v_1 = 0;
    v_2 = 0;
    v = [0 0]';
  end
  
  methods
    function obj = MK_Robot(initial_pos, property_vec, target_pos)
      %MK_ROBOT Construct an instance of this class
      
      %Set constant properties
      obj.m_c = property_vec(6);
      obj.m_o = property_vec(7);
      obj.m_k = property_vec(8);
      obj.l = property_vec(3);
      obj.l_c = property_vec(4);
      obj.d = property_vec(2);
      obj.r_c = property_vec(5);
      obj.R = property_vec(1);
      obj.k_1 = property_vec(9);
      obj.k_2 = property_vec(10);

      obj.B(1, 1) = (2*obj.d^2)/obj.R;
      obj.B(2, 2) = (2*obj.d^2)/obj.R;
      
      if size(target_pos, 1) == 1
        obj.target_pos = target_pos';
      else
        obj.target_pos = target_pos;
      end
      
      %Set variable properties
      obj.x = initial_pos(1);
      obj.y = initial_pos(2);
      
      if size(initial_pos, 1) == 1
        obj.pos = initial_pos';
      else
        obj.pos = initial_pos;
      end
      
      %Wheel angles
      obj.phi_1 = 0;
      obj.phi_2 = 0;
      obj.phi = [0, 0]';
      
      %velocity
      obj.v_1 = 0;
      obj.v_2 = 0;
      obj.v = [0 0]';

      %M is const since alpha is 0
      m11 = (obj.m_c + (4/3)*obj.m_o + 8*obj.m_k)*(obj.d^2) + obj.m_k*(obj.R^2) + obj.m_c*(0.5*(obj.r_c^2) + (1/3)*(obj.l_c^2));
      m21 = (obj.m_c + (2/3)*obj.m_o            )*(obj.d^2) - obj.m_k*(obj.R^2) - obj.m_c*(0.5*(obj.r_c^2) + (1/3)*(obj.l_c^2));
      m12 = (obj.m_c + (2/3)*obj.m_o            )*(obj.d^2) - obj.m_k*(obj.R^2) - obj.m_c*(0.5*(obj.r_c^2) + (1/3)*(obj.l_c^2));
      m22 = (obj.m_c + (4/3)*obj.m_o + 8*obj.m_k)*(obj.d^2) + obj.m_k*(obj.R^2) + obj.m_c*(0.5*(obj.r_c^2) + (1/3)*(obj.l_c^2));

      obj.M = [m11 m12; m21 m22];
    end
    
    function obj = move(obj, velocity, step)
      %move This method updates all non-constant members. Making the robot
      %move
      if obj.x == 10 || obj.y == 10
        fprintf("DONE\n");
      end
      
      n2_dR = 2 / obj.R;
      
      obj.phi_1 = obj.phi_1 + step*n2_dR*velocity(1);
      obj.phi_2 = obj.phi_2 + step*n2_dR*velocity(2);
      obj.phi = [obj.phi_1 obj.phi_2]';
      
      nR_d2d = obj.R / (2* obj.d); 
      
      theta = deg2rad(nR_d2d * (obj.phi_2 - obj.phi_1));
      
      old_pos = obj.pos;
      
      obj.x = obj.x + step * sum(velocity) * cos(theta);
      obj.y = obj.y + step * sum(velocity) * sin(theta);
      obj.pos = [obj.x obj.y]';
      
      obj.v = velocity;%(obj.pos - old_pos) / 0.001; 
      obj.v_1 = obj.v(1);
      obj.v_2 = obj.v(2);
    end
    
    function error_vec = get_error(obj)
      error_vec = obj.target_pos - obj.pos;
    end

    function N = calc_N(obj)
      const_expr = (4*(obj.d^2)*obj.k_1) / obj.R^2;

      n0 = const_expr * obj.v_1;
      n1 = const_expr * obj.v_2;

      N = [n0 n1]';
    end
  end
end

