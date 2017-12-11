%% Graphing the Path Planner Testbed Results
% Austin Hurst
clear all; clc; close all;

gazebo_sim = load("/home/austin/auvsi/autopilot_ws/telem.txt");
plot_sim_3d(gazebo_sim);
% plot_sim_3d();