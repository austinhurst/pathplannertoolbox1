function [] = plotCylinders(filename)
% This file takes in the cylinders from a file (N, E, radius, height) and plots the boundary on figure 1.
    % load in file
    cyls = load(filename);
    figure (1)
    hold on
    for i = 1:length(cyls(:,1))
        [Nc, Ec] = circle(cyls(i,1), cyls(i,2), cyls(i,3));
        fill(Ec, Nc,'r');
    end
end

function [Nc,Ec] = circle(N, E, r)
    th = 0:pi/15:2*pi;
    Ec = r*cos(th)+ E;
    Nc = r*sin(th)+ N;
end