clear;
clc;
close all;

initial_pos = [0 0];
property_vec = [0.254, 0.225, 0.12, 0.165, 0.035, 7, 0.5, 2, 0.05, 0.05];
target_pos = [10 10];

robot = MK_Robot(initial_pos, property_vec, target_pos);

K_11 = 0.2;
K_12 = 0.003;
K_21 = 1;
K_22 = 0.3;

L_11 = 1;
L_12 = 0;
L_21 = 0;
L_22 = 1;

Kappa = [K_11 K_12; K_21 K_22];
Lamda = [L_11 L_12; L_21 L_22];

mutation = 0.0001;
lr = 0.001; %learning rate
controller = VControl(robot, Kappa, Lamda, lr);

MAX_ITERATIONS = 100;
PERFORMANCE_ITERATIONS = 10;

performance = -inf;
perf_data = zeros(1, MAX_ITERATIONS);
perf_tick = 1:MAX_ITERATIONS;

iter = 1;
while iter <= MAX_ITERATIONS
  [area_p, area_v] = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 0);
  
  controller.Kappa(1,1) = controller.Kappa(1,1) + mutation;
  [area_2,~] = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 0);
  
  controller.Kappa(1,2) = controller.Kappa(1,2) + mutation;
  [area_3,~] = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 0);

  controller.Kappa(2,1) = controller.Kappa(2,1) + mutation;
  [area_4,~] = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 0);

  controller.Kappa(2,2) = controller.Kappa(2,2) + mutation;
  [area_5,~] = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 0);

  controller.Lamda(1,1) = controller.Lamda(1,1) + mutation;
  [~,area_6] = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 0);
  
  controller.Lamda(1,2) = controller.Lamda(1,2) + mutation;
  [~,area_7] = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 0);

  controller.Lamda(2,1) = controller.Lamda(2,1) + mutation;
  [~,area_8] = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 0);

  controller.Lamda(2,2) = controller.Lamda(2,2) + mutation;
  [~,area_9] = controller.SMC(robot, lr, PERFORMANCE_ITERATIONS, 0);
  
  controller.Kappa(1,1) = controller.Kappa(1,1) + lr * (area_p-area_2)/mutation;
  controller.Kappa(1,2) = controller.Kappa(1,2) + lr * (area_p-area_3)/mutation;
  controller.Kappa(2,1) = controller.Kappa(2,1) + lr * (area_p-area_4)/mutation;
  controller.Kappa(2,2) = controller.Kappa(2,2) + lr * (area_p-area_5)/mutation;

  controller.Lamda(1,1) = controller.Lamda(1,1) + lr * (area_v-area_6)/mutation;
  controller.Lamda(1,2) = controller.Lamda(1,2) + lr * (area_v-area_7)/mutation;
  controller.Lamda(2,1) = controller.Lamda(2,1) + lr * (area_v-area_8)/mutation;
  controller.Lamda(2,2) = controller.Lamda(2,2) + lr * (area_v-area_9)/mutation;

  performance = area_p^2 + area_v^2;
  perf_data(iter) = performance;
  
  iter = iter + 1;
end

controller.SMC(robot, 0.01, MAX_ITERATIONS, 1);

figure(2);
title("Performance Plot");
semilogy(perf_data);

fprintf("Optimal K is:\n");
controller.Kappa

fprintf("Optimal L is:\n");
controller.Lamda
