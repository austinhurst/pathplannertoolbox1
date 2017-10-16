%% Graphing the Path Planner Testbed Results
% Austin Hurst
clear; clc; close all;

plotBoundaries("output_boundaries.txt");
plotCylinders("output_cylinders.txt");
plotPrimaryWPS("output_primary_wps.txt");
plotTree("output_tree_0.txt");
plotPath("output_path.txt");