clear;
clc;
close all;

initial_pos = [0 0];
property_vec = [0.254, 0.225, 0.12, 0.165, 0.035, 7, 0.5, 2, 0.05, 0.05];
target_pos = [10 10];

robot = MK_Robot(initial_pos, property_vec, target_pos);

K_11 = 0.2;
K_12 = 0.03;
K_21 = 0.1;
K_22 = 0.01;

Kappa = [K_11 K_12; K_21 K_22];
mutation = 0.001;
lr = 0.01; %learning rate
controller = VControl(Kappa, lr);

MAX_ITERATIONS = 100;
PERFORMANCE_ITERATIONS = 20;

performance = -inf;

iter = 1;
while iter <= MAX_ITERATIONS && abs(performance) > 1e-3
  area_1 = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 1);
  
  controller.Kappa(1,1) = controller.Kappa(1,1) + mutation;
  area_2 = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 1);
  
  controller.Kappa(1,2) = controller.Kappa(1,2) + mutation;
  area_3 = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 1);

  controller.Kappa(2,1) = controller.Kappa(2,1) + mutation;
  area_4 = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 1);

  controller.Kappa(2,2) = controller.Kappa(2,2) + mutation;
  area_5 = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 1);
  
  controller.Kappa(1,1) = controller.Kappa(1,1) + lr * (area_1-area_2)/mutation;
  controller.Kappa(1,2) = controller.Kappa(1,2) + lr * (area_1-area_3)/mutation;
  controller.Kappa(2,1) = controller.Kappa(2,1) + lr * (area_1-area_4)/mutation;
  controller.Kappa(2,2) = controller.Kappa(2,2) + lr * (area_1-area_5)/mutation;

  performance = area_1;
  
  iter = iter + 1;
end

controller.SMC(robot, 0.01, MAX_ITERATIONS, 1);
